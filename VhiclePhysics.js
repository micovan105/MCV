/**
 * VehiclePhysics.js - Vehicle System (Vehicle Control & Vật lý xe)
 * Bộ quản lý điều khiển & vật lý xe cho Three.js
 * (Gia tốc, vận tốc tối đa, khối lượng, lật xe, va chạm, tuân theo danh sách vật cản)
 *
 * LƯU Ý: File này KHÔNG dùng tên class "MCVSystem" (trùng với MCVSystem của
 * MotionPhysics.js dùng cho nhân vật). Nếu trùng tên, file load sau sẽ ghi đè
 * class của file load trước trên window, gây lỗi khó hiểu (kể cả TDZ khi trùng
 * tên biến trong script.js). Vì vậy hệ thống xe dùng tên riêng: VehicleSystem.
 *
 * CÁCH DÙNG:
 *   const mcvXe = new VehicleSystem(camera);
 *   mcvXe.globalVatCan = [...danhSachMeshVatCan];
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
 * DÙNG SONG SONG VỚI HỆ THỐNG NHÂN VẬT (MotionPhysics.js):
 *   const mcvNguoi = new MCVSystem(camera);   // từ MotionPhysics.js
 *   const mcvXe    = new VehicleSystem(camera); // từ VehiclePhysics.js
 *   // Hai biến khác tên, hai class khác tên -> không còn đụng độ.
 */
 
class VehicleController {
    constructor(xe, camera, globalVatCan) {
        this.xe = xe;
        this.camera = camera;               // Không bắt buộc, chỉ dùng nếu muốn camera bám theo xe
        this.globalVatCan = globalVatCan;    // Mảng vật cản chung (tham chiếu từ MCVSystem)
 
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
 
        // --- CẤU HÌNH LẬT XE ---
        this.nguonggoclat = 0.55;       // Ngưỡng góc nghiêng (rad) để xe bị coi là lật (~31.5°)
        this.docamnghieng = 6.0;        // Tốc độ hồi phục góc nghiêng khi hết đánh lái
        this.hesonghiengtheotoc = 0.09; // Xe nghiêng nhiều hơn khi vào cua ở tốc độ cao
 
        // --- KÍCH THƯỚC VA CHẠM (BOUNDING BOX) ---
        this.kichthuocbox = new THREE.Vector3(2, 1.4, 4.2); // rộng, cao, dài mặc định
        this.autoBoxSize = true;
        this.vatcan = []; // Vật cản riêng của xe này (ngoài globalVatCan)
 
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
        };
 
        // --- TRẠNG THÁI NỘI BỘ ---
        this.tocdo = 0;                 // Tốc độ dọc thân xe hiện tại (âm = đang lùi)
        this.gocnghieng = 0;            // Góc nghiêng thân xe hiện tại (roll, rad)
        this.vantocy = 0;               // Vận tốc trục Y (nhảy dốc / rơi)
        this.trenmatdat = false;
        this.dangbilat = false;         // true khi xe đang trong trạng thái "lật"
        this.phimnhan = {};
 
        // Callback tuỳ chọn
        this.onVaCham = null;   // (lucVaCham) => {}
        this.onLat = null;      // () => {}
 
        this.clock = new THREE.Clock();
 
        // Vectors & cache tái sử dụng
        this.huongTienXe = new THREE.Vector3();
        this.trucdung = new THREE.Vector3(0, 1, 0);
        this.huongbanxuong = new THREE.Vector3(0, -1, 0);
 
        this.boxxe = new THREE.Box3();
        this.boxVatCanTemp = new THREE.Box3();
        this.raycaster = new THREE.Raycaster();
 
        this.capNhatKichThuocMacDinh();
        this.initEvents();
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
 
    getAllVatCan() {
        return [...this.globalVatCan, ...this.vatcan];
    }
 
    // Kiểm tra va chạm dựa trên AABB quanh tâm xe
    checkvacham() {
        const danhSachVatCan = this.getAllVatCan();
        if (danhSachVatCan.length === 0) return false;
 
        this.boxxe.setFromCenterAndSize(this.xe.position, this.kichthuocbox);
 
        for (let i = 0; i < danhSachVatCan.length; i++) {
            this.boxVatCanTemp.setFromObject(danhSachVatCan[i]);
            if (this.boxxe.intersectsBox(this.boxVatCanTemp)) return true;
        }
        return false;
    }
 
    // Xử lý phản ứng khi đâm vào vật cản: xe dội lại và mất tốc theo khối lượng
    xuLyVaCham(deltaTruoc) {
        const lucVaCham = Math.abs(this.tocdo) * (this.khoiluong / this.khoiluongthamchieu);
 
        // Lùi xe lại vị trí trước va chạm
        this.xe.position.sub(deltaTruoc);
 
        // Xe nặng mất tốc ít hơn, xe nhẹ mất tốc nhiều hơn (nảy lại một phần)
        const heSoDoiHuong = -0.15 * (this.khoiluongthamchieu / this.khoiluong);
        this.tocdo *= heSoDoiHuong;
 
        if (this.onVaCham) this.onVaCham(lucVaCham);
    }
 
    update() {
        const dt = Math.min(this.clock.getDelta(), 0.1);
        if (!this.status.active) return;
 
        const danhSachVatCan = this.getAllVatCan();
 
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
                const canLan = this.hesolan * dt;
                if (Math.abs(this.tocdo) <= canLan) this.tocdo = 0;
                else this.tocdo -= Math.sign(this.tocdo) * canLan;
            }
 
            if (dangPhanhTay) {
                const phanh = this.lucphanh * heSoQuanTinh * dt;
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
 
        // 4. DI CHUYỂN THEO HƯỚNG THÂN XE + VA CHẠM
        if (Math.abs(this.tocdo) > 0.001) {
            this.huongTienXe.set(0, 0, 1).applyQuaternion(this.xe.quaternion);
            const delta = this.huongTienXe.multiplyScalar(this.tocdo * dt);
 
            this.xe.position.add(delta);
 
            if (this.status.vacham && this.checkvacham()) {
                this.xuLyVaCham(delta);
            }
        }
 
        // 5. LEO DỐC / MẶT NGHIÊNG
        if (this.status.leodoc && Math.abs(this.tocdo) > 0.1 && danhSachVatCan.length > 0) {
            const viTriBan = this.xe.position.clone().add(new THREE.Vector3(0, 0.5, 0));
            this.raycaster.set(viTriBan, this.huongbanxuong);
            const vaChamDoc = this.raycaster.intersectObjects(danhSachVatCan);
 
            if (vaChamDoc.length > 0) {
                const doCaoMietDoc = vaChamDoc[0].point.y + (this.kichthuocbox.y / 2);
                const chenhLach = doCaoMietDoc - this.xe.position.y;
                if (chenhLach > 0 && chenhLach < 0.6 && this.trenmatdat) {
                    this.xe.position.y = doCaoMietDoc;
                }
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
    }
}
 
// --- CLASS QUẢN LÝ CHÍNH (dành riêng cho xe, không trùng MCVSystem của nhân vật) ---
class VehicleSystem {
    constructor(camera) {
        this.camera = camera;
        this.globalVatCan = []; // Vật cản chung toàn hệ thống
        this.controllers = new Map();
 
        // --- API DẠNG HÀM: mcv.vehicle(xemay) ---
        // Cho phép vừa gọi như hàm để lấy/tạo controller,
        // vừa gọi mcv.vehicle.off(xemay) / mcv.vehicle.run(xemay) mà không cần giữ biến controller.
        const he = this;
 
        const vehicleFn = function (xemay) {
            if (!xemay) return null;
            if (!he.controllers.has(xemay)) {
                const controller = new VehicleController(xemay, he.camera, he.globalVatCan);
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
 
    vehiclePhysics() {
        this.controllers.forEach((controller) => controller.update());
    }
}
 
if (typeof window !== 'undefined') {
    window.VehicleSystem = VehicleSystem;
}
