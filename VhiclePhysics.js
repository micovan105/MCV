/**
 * VehiclePhysics.js - Vehicle System (Vehicle Control & Vật lý xe)
 * Bộ quản lý điều khiển & vật lý xe cho Three.js
 * (Gia tốc, vận tốc tối đa, khối lượng, lật xe, va chạm, ma sát mặt đường,
 *  camera bám xe, nitro tạm thời, tuân theo danh sách vật cản)
 *
 * LƯU Ý: File này KHÔNG dùng tên class "MCVSystem" (trùng với MCVSystem của
 * MotionPhysics.js dùng cho nhân vật). Nếu trùng tên, file load sau sẽ ghi đè
 * class của file load trước trên window, gây lỗi khó hiểu (kể cả TDZ khi trùng
 * tên biến trong script.js). Vì vậy hệ thống xe dùng tên riêng: VehicleSystem.
 *
 * CÁCH DÙNG CƠ BẢN:
 *   const mcvXe = new VehicleSystem(camera);
 *   mcvXe.globalVatCan = [...danhSachMeshVatCan];   // có thể GÁN LẠI bất cứ lúc nào,
 *                                                    // các xe đã tạo trước đó vẫn nhận đúng danh sách mới
 *
 *   const xe = mcvXe.vehicle(xemay);       // Gán/tạo controller cho object xemay
 *   xe.tocdotoida = 30;
 *   xe.khoiluong  = 180;
 *
 *   mcvXe.vehicle.off(xemay);              // Tắt di chuyển của xemay (không cần giữ biến xe)
 *   mcvXe.vehicle.off(xemay, 'laixe');     // Chỉ tắt đánh lái
 *   mcvXe.vehicle.run(xemay);              // Bật lại toàn bộ
 *   mcvXe.vehicle.run(xemay, 'laixe');     // Bật lại riêng đánh lái
 *
 * Trong vòng lặp render:
 *   mcvXe.vehiclePhysics();
 *
 * TÍNH NĂNG NÂNG CAO:
 *   xe.status.camera = true;               // Bật camera tự bám theo xe (kiểu góc nhìn thứ 3)
 *   xe.cauHinhCamera.khoangCachSau = 10;    // Chỉnh khoảng cách/độ cao/độ trễ camera
 *
 *   xe.tangToc(1.6, 3);                    // Nitro: x1.6 tốc độ & gia tốc trong 3 giây rồi tự trả lại
 *
 *   // Ma sát mặt đường theo từng vật cản (mặc định heSoMaSat = 1):
 *   mesh.userData.heSoMaSat = 2.2;         // Bùn/cỏ: xe lăn chậm & phanh ngắn hơn
 *   mesh.userData.heSoMaSat = 0.35;        // Băng: xe trôi trượt, phanh dài hơn
 *
 *   // Vật cản di chuyển (không cache bounding box, luôn tính lại mỗi frame):
 *   mesh.userData.dongtinh = true;
 *   // Nếu bạn tự di chuyển 1 vật cản "tĩnh" (không có dongtinh), nhớ làm mới cache:
 *   mcvXe.lamMoiBoxVatCan(mesh);           // hoặc mcvXe.xoaCacheBox() để xoá hết
 *
 * DÙNG SONG SONG VỚI HỆ THỐNG NHÂN VẬT (MotionPhysics.js):
 *   const mcvNguoi = new MCVSystem(camera);   // từ MotionPhysics.js
 *   const mcvXe    = new VehicleSystem(camera); // từ VehiclePhysics.js
 *   // Hai biến khác tên, hai class khác tên -> không còn đụng độ.
 */
 
class VehicleController {
    constructor(xe, owner) {
        this.xe = xe;
        this.owner = owner || null;         // Tham chiếu VehicleSystem cha (để lấy globalVatCan "sống" & cache box dùng chung)
        this.camera = this.owner ? this.owner.camera : null; // Không bắt buộc, chỉ dùng nếu bật status.camera
 
        // Nếu tạo controller độc lập (không qua VehicleSystem), dùng mảng nội bộ riêng
        this._globalVatCanDoclap = [];
 
        // --- CẤU HÌNH VẬT LÝ XE (chỉnh trực tiếp qua xe.<tên>) ---
        this.giatoc = 8.0;              // Gia tốc khi ga (m/s²)
        this.giatoclui = 4.0;           // Gia tốc khi lùi (m/s²)
        this.tocdotoida = 25.0;         // Vận tốc tối đa tiến (m/s)
        this.tocdoluitoida = 8.0;       // Vận tốc tối đa lùi (m/s)
        this.lucphanh = 18.0;           // Lực phanh (m/s²)
        this.hesolan = 2.5;             // Cản lăn / ma sát mặt đường (m/s²) khi buông ga
        this.tocdovolang = 2.2;         // Tốc độ đánh lái (rad/s) ở vận tốc thấp
        this.trongluc = 20.0;           // Gia tốc trọng lực
 
        // Khối lượng xe (kg) - ảnh hưởng quán tính tăng/giảm tốc & lực va chạm
        this.khoiluong = 1000;
        this.khoiluongthamchieu = 1000; // Mốc tham chiếu để tính hệ số quán tính tương đối
 
        // --- CẤU HÌNH VA CHẠM ---
        this.heSoNayVaCham = 0.15;      // Hệ số nảy lại khi đâm vật cản (0 = dừng khựng, càng lớn càng nảy mạnh)
 
        // --- CẤU HÌNH MA SÁT MẶT ĐƯỜNG THEO userData.heSoMaSat CỦA VẬT CẢN BÊN DƯỚI ---
        this.heSoMaSatMacDinh = 1.0;    // Dùng khi không đứng trên vật cản nào có heSoMaSat riêng
        this.heSoMaSatHienTai = 1.0;    // Đọc để biết mặt đường hiện tại (chỉ đọc, tự cập nhật mỗi frame)
 
        // --- CẤU HÌNH LẬT XE ---
        this.nguonggoclat = 0.55;       // Ngưỡng góc nghiêng (rad) để xe bị coi là lật (~31.5°)
        this.docamnghieng = 6.0;        // Tốc độ hồi phục góc nghiêng khi hết đánh lái
        this.hesonghiengtheotoc = 0.09; // Xe nghiêng nhiều hơn khi vào cua ở tốc độ cao
 
        // --- KÍCH THƯỚC VA CHẠM (BOUNDING BOX) ---
        this.kichthuocbox = new THREE.Vector3(2, 1.4, 4.2); // rộng, cao, dài mặc định
        this.autoBoxSize = true;
        this.vatcan = []; // Vật cản riêng của xe này (ngoài globalVatCan)
 
        // --- CẤU HÌNH CAMERA BÁM THEO XE (chỉ hoạt động khi status.camera = true) ---
        this.cauHinhCamera = {
            khoangCachSau: 8,   // Khoảng cách phía sau xe
            doCao: 3.5,         // Độ cao so với xe
            doTre: 5,           // Độ trễ bám theo (càng lớn càng mượt nhưng chậm bám hơn)
        };
 
        // --- CÔNG TẮC BẬT/TẮT TÍNH NĂNG ---
        this.status = {
            active: true,
            dichuyen: true,
            laixe: true,
            phanh: true,
            trongluc: true,
            leodoc: true,
            vacham: true,
            lat: true,       // Bật/tắt mô phỏng lật xe
            camera: false,   // Bật/tắt camera tự bám theo xe
        };
 
        // --- TRẠNG THÁI NỘI BỘ ---
        this.tocdo = 0;                 // Tốc độ dọc thân xe hiện tại (âm = đang lùi)
        this.gocnghieng = 0;            // Góc nghiêng thân xe hiện tại (roll, rad)
        this.vantocy = 0;               // Vận tốc trục Y (nhảy dốc / rơi)
        this.trenmatdat = false;
        this.dangbilat = false;         // true khi xe đang trong trạng thái "lật"
        this.phimnhan = {};
 
        this._giaTocGoc = null;         // Lưu giá trị gốc khi dùng tangToc()
        this._tocDoToiDaGoc = null;
        this._boostTimeout = null;
 
        // Callback tuỳ chọn
        this.onVaCham = null;   // (lucVaCham, thongTin) => {}   thongTin = { truc, capTruc }
        this.onLat = null;      // () => {}
 
        this.clock = new THREE.Clock();
 
        // Vectors & cache tái sử dụng
        this.huongTienXe = new THREE.Vector3();
        this.trucdung = new THREE.Vector3(0, 1, 0);
        this.huongbanxuong = new THREE.Vector3(0, -1, 0);
 
        this.boxxe = new THREE.Box3();
        this.boxVatCanTemp = new THREE.Box3();
        this.raycaster = new THREE.Raycaster();
        this._viTriCameraTam = new THREE.Vector3();
        this._lookAtTam = new THREE.Vector3();
 
        this.capNhatKichThuocMacDinh();
        this.initEvents();
    }
 
    // Cho phép đọc/gán globalVatCan trực tiếp trên từng controller.
    // - Nếu controller thuộc 1 VehicleSystem: đọc/ghi thẳng vào mảng "sống" của hệ thống đó
    //   (mọi xe dùng chung 1 nguồn, gán lại ở đâu cũng đồng bộ ngay lập tức).
    // - Nếu controller được tạo độc lập (không qua VehicleSystem): dùng mảng riêng của nó.
    get globalVatCan() {
        return this.owner ? this.owner.globalVatCan : this._globalVatCanDoclap;
    }
    set globalVatCan(mang) {
        if (this.owner) this.owner.globalVatCan = mang;
        else this._globalVatCanDoclap = mang;
    }
 
    capNhatKichThuocMacDinh() {
        if (!this.xe) return;
        const tempBox = new THREE.Box3().setFromObject(this.xe);
        const size = new THREE.Vector3();
        tempBox.getSize(size);
        if (size.x > 0 && size.y > 0 && size.z > 0) {
            this.kichthuocbox.copy(size);
        }
    }
 
    setBoxSize(x, y, z) {
        this.autoBoxSize = false;
        this.kichthuocbox.set(x, y, z);
        return this;
    }
 
    initEvents() {
        this._onKeyDown = (e) => { this.phimnhan[e.code] = true; };
        this._onKeyUp = (e) => { this.phimnhan[e.code] = false; };
        window.addEventListener('keydown', this._onKeyDown);
        window.addEventListener('keyup', this._onKeyUp);
    }
 
    destroy() {
        window.removeEventListener('keydown', this._onKeyDown);
        window.removeEventListener('keyup', this._onKeyUp);
        if (this._boostTimeout) clearTimeout(this._boostTimeout);
    }
 
    // TẮT: off() tắt hết, off('laixe') chỉ tắt đánh lái, v.v.
    off(feature) {
        if (!feature) this.status.active = false;
        else if (this.status.hasOwnProperty(feature)) this.status[feature] = false;
        return this;
    }
 
    // BẬT: run() bật lại hết, run('laixe') chỉ bật lại đánh lái, v.v.
    run(feature) {
        if (!feature) { this.status.active = true; this.clock.start(); }
        else if (this.status.hasOwnProperty(feature)) this.status[feature] = true;
        return this;
    }
 
    // NITRO / TĂNG TỐC TẠM THỜI: nhân giatoc & tocdotoida trong "giay" giây rồi tự trả về gốc.
    // Gọi lại lúc đang boost sẽ reset lại thời gian đếm ngược (không cộng dồn hệ số).
    tangToc(heSo = 1.5, giay = 3) {
        if (this._boostTimeout) clearTimeout(this._boostTimeout);
        if (this._giaTocGoc === null) {
            this._giaTocGoc = this.giatoc;
            this._tocDoToiDaGoc = this.tocdotoida;
        }
        this.giatoc = this._giaTocGoc * heSo;
        this.tocdotoida = this._tocDoToiDaGoc * heSo;
        this._boostTimeout = setTimeout(() => {
            this.giatoc = this._giaTocGoc;
            this.tocdotoida = this._tocDoToiDaGoc;
            this._giaTocGoc = null;
            this._tocDoToiDaGoc = null;
            this._boostTimeout = null;
        }, giay * 1000);
        return this;
    }
 
    getAllVatCan() {
        return [...this.globalVatCan, ...this.vatcan];
    }
 
    // Lấy Box3 thế giới của 1 vật cản - dùng cache dùng chung (qua owner) nếu có, để tránh
    // tính lại setFromObject() cho từng vật cản tĩnh ở mỗi frame, mỗi xe.
    _layBoxCuaVatCan(obj) {
        if (this.owner) return this.owner.layBoxVatCan(obj);
        this.boxVatCanTemp.setFromObject(obj);
        return this.boxVatCanTemp;
    }
 
    // Kiểm tra va chạm dựa trên AABB quanh tâm xe
    checkvacham() {
        const danhSachVatCan = this.getAllVatCan();
        if (danhSachVatCan.length === 0) return false;
 
        this.boxxe.setFromCenterAndSize(this.xe.position, this.kichthuocbox);
 
        for (let i = 0; i < danhSachVatCan.length; i++) {
            const box = this._layBoxCuaVatCan(danhSachVatCan[i]);
            if (this.boxxe.intersectsBox(box)) return true;
        }
        return false;
    }
 
    // Xử lý phản ứng khi đâm vào vật cản: xe mất tốc theo khối lượng & nảy lại nhẹ.
    // Vị trí đã được hoàn tác theo từng trục ở update() trước khi gọi hàm này,
    // nên ở đây chỉ cần xử lý tốc độ + báo callback.
    xuLyVaCham(vaChamTrucX, vaChamTrucZ) {
        const lucVaCham = Math.abs(this.tocdo) * (this.khoiluong / this.khoiluongthamchieu);
 
        // Xe nặng mất tốc ít hơn, xe nhẹ mất tốc nhiều hơn (nảy lại một phần)
        const heSoDoiHuong = -this.heSoNayVaCham * (this.khoiluongthamchieu / this.khoiluong);
        this.tocdo *= heSoDoiHuong;
 
        if (this.onVaCham) {
            this.onVaCham(lucVaCham, {
                truc: vaChamTrucX && vaChamTrucZ ? 'ca-hai' : (vaChamTrucX ? 'x' : 'z'),
                capTruc: vaChamTrucX && vaChamTrucZ,
            });
        }
    }
 
    // Bắn tia xuống dưới gầm xe để: (1) hỗ trợ leo dốc, (2) đọc hệ số ma sát mặt đường
    // từ userData.heSoMaSat của vật cản đang đứng lên trên (nếu có).
    _capNhatMatDatVaMaSat(danhSachVatCan) {
        this.heSoMaSatHienTai = this.heSoMaSatMacDinh;
        if (!this.status.leodoc || danhSachVatCan.length === 0) return null;
 
        const viTriBan = this.xe.position.clone().add(new THREE.Vector3(0, 0.5, 0));
        this.raycaster.set(viTriBan, this.huongbanxuong);
        const vaChamDoc = this.raycaster.intersectObjects(danhSachVatCan);
 
        if (vaChamDoc.length > 0) {
            const vatCanBenDuoi = vaChamDoc[0].object;
            if (vatCanBenDuoi.userData && typeof vatCanBenDuoi.userData.heSoMaSat === 'number') {
                this.heSoMaSatHienTai = vatCanBenDuoi.userData.heSoMaSat;
            }
        }
        return vaChamDoc;
    }
 
    update() {
        const dt = Math.min(this.clock.getDelta(), 0.1);
        if (!this.status.active) return;
 
        const danhSachVatCan = this.getAllVatCan();
 
        // 0. MẶT ĐƯỜNG / MA SÁT (dùng chung tia bắn xuống cho cả leo dốc lẫn ma sát)
        const vaChamDocDaTinh = this._capNhatMatDatVaMaSat(danhSachVatCan);
        const heSoMaSat = this.heSoMaSatHienTai;
 
        // 1. GA / LÙI / PHANH
        if (this.status.dichuyen && !this.dangbilat) {
            const dangGa = this.phimnhan["KeyW"] || this.phimnhan["ArrowUp"];
            const dangLui = this.phimnhan["KeyS"] || this.phimnhan["ArrowDown"];
            const dangPhanhTay = this.status.phanh && this.phimnhan["Space"];
 
            // Hệ số quán tính: xe càng nặng, tăng/giảm tốc càng chậm
            const heSoQuanTinh = this.khoiluongthamchieu / this.khoiluong;
 
            if (dangGa) {
                this.tocdo += this.giatoc * heSoQuanTinh * dt;
            } else if (dangLui) {
                this.tocdo -= this.giatoclui * heSoQuanTinh * dt;
            } else {
                // Cản lăn: mặt đường trơn (heSoMaSat < 1) khiến xe trôi xa hơn khi buông ga
                const canLan = this.hesolan * heSoMaSat * dt;
                if (Math.abs(this.tocdo) <= canLan) this.tocdo = 0;
                else this.tocdo -= Math.sign(this.tocdo) * canLan;
            }
 
            if (dangPhanhTay) {
                // Mặt đường trơn cũng làm quãng đường phanh dài hơn
                const phanh = this.lucphanh * heSoMaSat * heSoQuanTinh * dt;
                if (Math.abs(this.tocdo) <= phanh) this.tocdo = 0;
                else this.tocdo -= Math.sign(this.tocdo) * phanh;
            }
 
            this.tocdo = THREE.MathUtils.clamp(this.tocdo, -this.tocdoluitoida, this.tocdotoida);
        }
 
        // 2. ĐÁNH LÁI (chỉ hiệu quả khi xe đang di chuyển)
        let dangQuay = 0;
        if (this.status.laixe && !this.dangbilat && Math.abs(this.tocdo) > 0.05) {
            const heSoToc = THREE.MathUtils.clamp(Math.abs(this.tocdo) / this.tocdotoida, 0.15, 1);
            const chieu = this.tocdo >= 0 ? 1 : -1; // lùi thì bẻ lái ngược lại
 
            if (this.phimnhan["KeyA"] || this.phimnhan["ArrowLeft"]) dangQuay += 1;
            if (this.phimnhan["KeyD"] || this.phimnhan["ArrowRight"]) dangQuay -= 1;
 
            if (dangQuay !== 0) {
                this.xe.rotation.y += dangQuay * chieu * this.tocdovolang * heSoToc * dt;
            }
        }
 
        // Chuẩn hoá góc xoay Y để tránh số float phình to sau thời gian dài chạy
        if (this.xe.rotation.y > Math.PI * 2 || this.xe.rotation.y < -Math.PI * 2) {
            this.xe.rotation.y = this.xe.rotation.y % (Math.PI * 2);
        }
 
        // 3. TÍNH GÓC NGHIÊNG (ROLL) & KIỂM TRA LẬT XE
        if (this.status.lat) {
            const nghiengMucTieu = -dangQuay * Math.min(Math.abs(this.tocdo) * this.hesonghiengtheotoc, 0.5);
            this.gocnghieng += (nghiengMucTieu - this.gocnghieng) * Math.min(this.docamnghieng * dt, 1);
 
            if (!this.dangbilat && Math.abs(this.gocnghieng) >= this.nguonggoclat) {
                this.dangbilat = true;
                this.tocdo *= 0.1; // Mất gần hết tốc độ khi lật
                if (this.onLat) this.onLat();
            }
 
            // Xe tự "đứng dậy" chậm rãi sau khi lật (có thể tắt bằng status.lat = false)
            if (this.dangbilat) {
                this.gocnghieng += (0 - this.gocnghieng) * Math.min(1.0 * dt, 1);
                if (Math.abs(this.gocnghieng) < 0.02) {
                    this.gocnghieng = 0;
                    this.dangbilat = false;
                }
            }
 
            this.xe.rotation.z = this.gocnghieng;
        }
 
        // 4. DI CHUYỂN THEO HƯỚNG THÂN XE + VA CHẠM (trượt theo từng trục thay vì dừng khựng)
        if (Math.abs(this.tocdo) > 0.001) {
            this.huongTienXe.set(0, 0, 1).applyQuaternion(this.xe.quaternion);
            const full = this.huongTienXe.multiplyScalar(this.tocdo * dt);
 
            if (this.status.vacham) {
                let vaChamTrucX = false;
                let vaChamTrucZ = false;
 
                // Thử trục X trước, nếu chạm thì hoàn tác riêng trục X
                if (Math.abs(full.x) > 0) {
                    this.xe.position.x += full.x;
                    if (this.checkvacham()) { this.xe.position.x -= full.x; vaChamTrucX = true; }
                }
                // Rồi thử trục Z độc lập -> cho phép xe "trượt" dọc theo tường thay vì khựng lại
                if (Math.abs(full.z) > 0) {
                    this.xe.position.z += full.z;
                    if (this.checkvacham()) { this.xe.position.z -= full.z; vaChamTrucZ = true; }
                }
 
                if (vaChamTrucX || vaChamTrucZ) this.xuLyVaCham(vaChamTrucX, vaChamTrucZ);
            } else {
                this.xe.position.x += full.x;
                this.xe.position.z += full.z;
            }
        }
 
        // 5. LEO DỐC / MẶT NGHIÊNG (dùng lại kết quả tia bắn xuống đã tính ở bước 0)
        if (this.status.leodoc && Math.abs(this.tocdo) > 0.1 && vaChamDocDaTinh && vaChamDocDaTinh.length > 0) {
            const doCaoMietDoc = vaChamDocDaTinh[0].point.y + (this.kichthuocbox.y / 2);
            const chenhLach = doCaoMietDoc - this.xe.position.y;
            if (chenhLach > 0 && chenhLach < 0.6 && this.trenmatdat) {
                this.xe.position.y = doCaoMietDoc;
            }
        }
 
        // 6. TRỌNG LỰC / TRỤC Y
        if (this.status.trongluc) {
            this.vantocy -= this.trongluc * dt;
            const deltaY = this.vantocy * dt;
            this.xe.position.y += deltaY;
 
            if (this.checkvacham()) {
                this.xe.position.y -= deltaY;
                if (this.vantocy < 0) this.trenmatdat = true;
                this.vantocy = 0;
            } else {
                this.trenmatdat = false;
            }
 
            if (this.xe.position.y < -50) {
                this.xe.position.set(0, 10, 0);
                this.tocdo = 0;
                this.vantocy = 0;
                this.gocnghieng = 0;
                this.dangbilat = false;
                this.trenmatdat = false;
            }
        }
 
        // 7. CAMERA BÁM THEO XE (chỉ chạy khi bật status.camera và có camera)
        this._capNhatCamera(dt);
    }
 
    _capNhatCamera(dt) {
        if (!this.status.camera || !this.camera) return;
 
        const cfg = this.cauHinhCamera;
        this._viTriCameraTam
            .set(0, cfg.doCao, -cfg.khoangCachSau)
            .applyQuaternion(this.xe.quaternion)
            .add(this.xe.position);
 
        // Độ trễ mượt, không phụ thuộc framerate
        const heSoLerp = 1 - Math.pow(0.001, dt * (cfg.doTre / 5));
        this.camera.position.lerp(this._viTriCameraTam, THREE.MathUtils.clamp(heSoLerp, 0, 1));
 
        this._lookAtTam.copy(this.xe.position).add(this.trucdung);
        this.camera.lookAt(this._lookAtTam);
    }
}
 
// --- CLASS QUẢN LÝ CHÍNH (dành riêng cho xe, không trùng MCVSystem của nhân vật) ---
class VehicleSystem {
    constructor(camera) {
        this.camera = camera;
        this.globalVatCan = []; // Vật cản chung toàn hệ thống (có thể GÁN LẠI bất cứ lúc nào)
        this.controllers = new Map();
 
        // Cache Box3 thế giới cho vật cản "tĩnh" (dùng chung giữa mọi xe, tránh tính lại mỗi frame).
        // Vật cản có userData.dongtinh === true sẽ KHÔNG được cache, luôn tính lại mỗi lần kiểm tra.
        this._boxCache = new Map();
 
        // --- API DẠNG HÀM: mcv.vehicle(xemay) ---
        // Cho phép vừa gọi như hàm để lấy/tạo controller,
        // vừa gọi mcv.vehicle.off(xemay) / mcv.vehicle.run(xemay) mà không cần giữ biến controller.
        const he = this;
 
        const vehicleFn = function (xemay) {
            if (!xemay) return null;
            if (!he.controllers.has(xemay)) {
                const controller = new VehicleController(xemay, he);
                he.controllers.set(xemay, controller);
            }
            return he.controllers.get(xemay);
        };
 
        // mcv.vehicle.off(xemay [, feature]) -> tắt di chuyển (hoặc 1 tính năng cụ thể) của xemay
        vehicleFn.off = function (xemay, feature) {
            const controller = vehicleFn(xemay); // tự tạo nếu chưa có, rồi tắt luôn
            return controller ? controller.off(feature) : null;
        };
 
        // mcv.vehicle.run(xemay [, feature]) -> bật lại di chuyển (hoặc 1 tính năng cụ thể) của xemay
        vehicleFn.run = function (xemay, feature) {
            const controller = he.controllers.get(xemay);
            return controller ? controller.run(feature) : null;
        };
 
        // mcv.vehicle.remove(xemay) -> gỡ hẳn xe khỏi hệ thống (dừng lắng nghe phím...)
        vehicleFn.remove = function (xemay) {
            const controller = he.controllers.get(xemay);
            if (controller) {
                controller.destroy();
                he.controllers.delete(xemay);
            }
        };
 
        this.vehicle = vehicleFn;
    }
 
    // Lấy Box3 thế giới của 1 vật cản, dùng cache nếu vật cản là "tĩnh"
    // (mặc định mọi vật cản được coi là tĩnh, trừ khi đặt userData.dongtinh = true).
    layBoxVatCan(obj) {
        const laDongTinh = !!(obj.userData && obj.userData.dongtinh === true);
        if (!laDongTinh) {
            const cached = this._boxCache.get(obj);
            if (cached) return cached;
        }
        const box = new THREE.Box3().setFromObject(obj);
        if (!laDongTinh) this._boxCache.set(obj, box);
        return box;
    }
 
    // Xoá cache box của 1 vật cản cụ thể - gọi sau khi bạn tự di chuyển/biến đổi
    // một vật cản vốn được coi là "tĩnh" (không có userData.dongtinh = true).
    lamMoiBoxVatCan(obj) {
        this._boxCache.delete(obj);
    }
 
    // Xoá toàn bộ cache box - gọi sau khi thay đổi hàng loạt globalVatCan hoặc load lại map.
    xoaCacheBox() {
        this._boxCache.clear();
    }
 
    vehiclePhysics() {
        this.controllers.forEach((controller) => controller.update());
    }
}
 
if (typeof window !== 'undefined') {
    window.VehicleSystem = VehicleSystem;
}
