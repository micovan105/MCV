/**
 * MotionPhysics.js - MCV System
 * Bộ quản lý di chuyển & vật lý đơn giản cho Three.js (Fixed Collision & Dynamic BoundingBox)
 *
 * FIX (bản này): sửa lỗi nhân vật bị chặn bởi 1 box "phồng to / lệch" khi vật cản là 1
 * object đang XOAY (ví dụ 1 chiếc xe từ VehiclePhysics.js đang đánh lái chéo góc).
 *
 * Nguyên nhân cũ: checkvacham() luôn tính box của MỌI vật cản bằng Box3.setFromObject(),
 * hàm này trả về AABB theo trục THẾ GIỚI. Khi vật cản xoay 1 góc bất kỳ (không phải bội
 * số 90°), AABB bao quanh nó luôn to hơn hình dạng thật (tối đa ~1.41 lần ở góc 45°)
 * -> nhân vật cảm giác đụng phải 1 cái hộp vô hình to/lệch hơn vật thể thật.
 *
 * Cách sửa: cho phép 1 vật cản khai báo "hộp định hướng" (OOB) qua:
 *   mesh.userData.oob = { kichthuoc: new THREE.Vector3(rong, cao, dai) };
 * Khi đó nhân vật sẽ kiểm tra va chạm bằng SAT (AABB nhân vật vs OBB vật cản, xoay theo
 * mesh.rotation.y) thay vì AABB-vs-AABB thẳng trục thế giới. VehiclePhysics.js (bản đã
 * fix) tự động gắn userData.oob này cho chiếc xe, nên chỉ cần dùng chung globalVatCan là
 * 2 hệ thống tương thích ngay, không cần cấu hình gì thêm. Vật cản KHÔNG khai báo oob
 * (vật cản tĩnh, không xoay) vẫn dùng cách kiểm tra AABB cũ như trước, không đổi hành vi.
 */

class MotionController {
    constructor(nhanvat, camera, globalVatCan) {
        this.nhanvat = nhanvat;
        this.camera = camera;
        this.globalVatCan = globalVatCan;

        // --- CẤU HÌNH VẬT LÝ (Tính theo m/s và m/s²) ---
        this.tocdodichuyen = 7.0;       // Tốc độ đi bộ (m/s)
        this.tocdochaynhanh = 13.0;     // Tốc độ chạy nhanh (m/s)
        this.lucnhay = 11.0;            // Vận tốc nhảy đầu (m/s)
        this.trongluc = 20.0;           // Gia tốc trọng lực (đã điều chỉnh tối ưu chống văng)
        this.doMaSat = 14.0;             // Độ dừng/quán tính

        // Tự động tính toán kích thước thực tế của nhân vật nếu có
        this.kichthuocbox = new THREE.Vector3(1, 1.8, 1);
        this.offsetBox = new THREE.Vector3(0, 0, 0); // Độ lệch tâm box so với nhanvat.position
        this.autoBoxSize = true; // Mặc định tự động tính theo model

        this.vatcan = []; // Vật cản riêng

        // --- BỘ CÔNG TẮC BẬT / TẮT TÍNH NĂNG ---
        this.status = {
            active: true,
            dichuyen: true,
            nhay: true,
            chay: true,
            trongluc: true,
            leodoc: true
        };

        // --- TRẠNG THÁI NỘI BỘ & VẬN TỐC ---
        this.vanTocHienTai = new THREE.Vector3(); 
        this.trenmatdat = false;
        this.phimnhan = {};

        // Clock quản lý Delta Time
        this.clock = new THREE.Clock();

        // Vectors & Cache tái sử dụng
        this.huongtien = new THREE.Vector3();
        this.huongngang = new THREE.Vector3();
        this.huongMuonDi = new THREE.Vector3();
        this.trucdung = new THREE.Vector3(0, 1, 0);
        this.huongbanxuong = new THREE.Vector3(0, -1, 0);
        
        this._tamNhanVat = new THREE.Vector3(); // Tâm box nhân vật (position + offsetBox)
        this.boxnhanvat = new THREE.Box3();
        this.boxVatCanTemp = new THREE.Box3(); 
        this.raycaster = new THREE.Raycaster();

        // --- CACHE CHO KIỂM TRA VA CHẠM DẠNG "AABB NHÂN VẬT vs OBB VẬT CẢN XOAY" (SAT 2D) ---
        // Dùng khi vật cản có khai báo userData.oob (ví dụ chiếc xe từ VehiclePhysics.js).
        // Mảng tái sử dụng mỗi frame để tránh cấp phát object mới liên tục (đỡ GC).
        this._gocNhanVatXZ = [{ x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }];
        this._gocVatCanOOB = [{ x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }];
        this._cacTrucSAT = [{ x: 1, y: 0 }, { x: 0, y: 1 }, { x: 1, y: 0 }, { x: 0, y: 1 }];

        // Tự động đo kích thước ban đầu của nhân vật
        this.capNhatKichThuocMacDinh();
        this.initEvents();
    }

    // LỆNH MỚI: Tự động đo BoundingBox của Mesh/Group nhân vật
    capNhatKichThuocMacDinh() {
        if (!this.nhanvat) return;
        const tempBox = new THREE.Box3().setFromObject(this.nhanvat);
        const size = new THREE.Vector3();
        tempBox.getSize(size);
        
        // Nếu nhân vật có kích thước hợp lệ (lớn hơn 0)
        if (size.x > 0 && size.y > 0 && size.z > 0) {
            this.kichthuocbox.copy(size);
        }
    }

    // LỆNH MỚI: Hàm đặt kích thước Box nhân vật thủ công theo ý muốn
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

    off(feature) {
        if (!feature) {
            this.status.active = false;
        } else if (this.status.hasOwnProperty(feature)) {
            this.status[feature] = false;
        }
        return this;
    }

    run(feature) {
        if (!feature) {
            this.status.active = true;
            this.clock.start();
        } else if (this.status.hasOwnProperty(feature)) {
            this.status[feature] = true;
        }
        return this;
    }

    getAllVatCan() {
        return [...this.globalVatCan, ...this.vatcan];
    }

    // Kiểm tra AABB (box nhân vật, không xoay) va chạm với 1 OBB (box vật cản xoay theo
    // yaw của chính nó) bằng SAT trên mặt phẳng XZ, cộng kiểm tra riêng trục Y.
    // tamVatCan: THREE.Vector3 (world position, coi là tâm box)
    // kichthuocVatCan: THREE.Vector3 (rộng, cao, dài - kích thước LOCAL, chưa xoay)
    // yaw: góc xoay quanh trục Y (rad)
    _kiemtraAABBvsOBB(tamVatCan, kichthuocVatCan, yaw) {
        // 1. Trục Y kiểm tra riêng (yaw không ảnh hưởng chiều cao)
        const yMinVC = tamVatCan.y - kichthuocVatCan.y / 2;
        const yMaxVC = tamVatCan.y + kichthuocVatCan.y / 2;
        if (this.boxnhanvat.max.y < yMinVC || yMaxVC < this.boxnhanvat.min.y) return false;

        // 2. SAT 2D trên mặt phẳng XZ
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        const hx = kichthuocVatCan.x / 2;
        const hz = kichthuocVatCan.z / 2;

        // 4 góc cục bộ của vật cản trước khi xoay: (-hx,-hz) (hx,-hz) (hx,hz) (-hx,hz)
        const lx = [-hx, hx, hx, -hx];
        const lz = [-hz, -hz, hz, hz];
        for (let i = 0; i < 4; i++) {
            this._gocVatCanOOB[i].x = tamVatCan.x + lx[i] * cos + lz[i] * sin;
            this._gocVatCanOOB[i].y = tamVatCan.z - lx[i] * sin + lz[i] * cos;
        }

        // 4 góc của box nhân vật (AABB, không xoay)
        this._gocNhanVatXZ[0].x = this.boxnhanvat.min.x; this._gocNhanVatXZ[0].y = this.boxnhanvat.min.z;
        this._gocNhanVatXZ[1].x = this.boxnhanvat.max.x; this._gocNhanVatXZ[1].y = this.boxnhanvat.min.z;
        this._gocNhanVatXZ[2].x = this.boxnhanvat.max.x; this._gocNhanVatXZ[2].y = this.boxnhanvat.max.z;
        this._gocNhanVatXZ[3].x = this.boxnhanvat.min.x; this._gocNhanVatXZ[3].y = this.boxnhanvat.max.z;

        // 4 trục cần thử: 2 trục thế giới (box nhân vật) + 2 trục cục bộ của vật cản (theo yaw)
        this._cacTrucSAT[0].x = 1; this._cacTrucSAT[0].y = 0;
        this._cacTrucSAT[1].x = 0; this._cacTrucSAT[1].y = 1;
        this._cacTrucSAT[2].x = cos; this._cacTrucSAT[2].y = -sin;
        this._cacTrucSAT[3].x = sin; this._cacTrucSAT[3].y = cos;

        for (let i = 0; i < 4; i++) {
            const truc = this._cacTrucSAT[i];

            let minA = Infinity, maxA = -Infinity;
            for (let k = 0; k < 4; k++) {
                const d = this._gocNhanVatXZ[k].x * truc.x + this._gocNhanVatXZ[k].y * truc.y;
                if (d < minA) minA = d;
                if (d > maxA) maxA = d;
            }

            let minB = Infinity, maxB = -Infinity;
            for (let k = 0; k < 4; k++) {
                const d = this._gocVatCanOOB[k].x * truc.x + this._gocVatCanOOB[k].y * truc.y;
                if (d < minB) minB = d;
                if (d > maxB) maxB = d;
            }

            // Tìm được 1 trục tách rời -> chắc chắn KHÔNG va chạm, dừng sớm
            if (maxA < minB || maxB < minA) return false;
        }

        // Không trục nào tách được -> có va chạm
        return true;
    }

    // Kiểm tra va chạm: vật cản có userData.oob (khai báo là "hộp xoay") thì dùng SAT
    // (AABB nhân vật vs OBB vật cản); vật cản thường thì vẫn dùng AABB-vs-AABB như cũ.
    checkvacham() {
        const danhSachVatCan = this.getAllVatCan();
        if (danhSachVatCan.length === 0) return false;

        // Cập nhật vị trí Box3 chuẩn xác dựa trên vị trí nhân vật (+ độ lệch tâm nếu có)
        this._tamNhanVat.copy(this.nhanvat.position).add(this.offsetBox);
        this.boxnhanvat.setFromCenterAndSize(this._tamNhanVat, this.kichthuocbox);

        for (let i = 0; i < danhSachVatCan.length; i++) {
            const obj = danhSachVatCan[i];

            // Vật cản khai báo OOB (ví dụ 1 chiếc xe từ VehiclePhysics.js đang xoay theo
            // yaw) -> kiểm tra bằng SAT, tránh bị "phồng to" AABB khi vật cản xoay chéo góc.
            const oob = obj.userData && obj.userData.oob;
            if (oob && oob.kichthuoc) {
                if (this._kiemtraAABBvsOBB(obj.position, oob.kichthuoc, obj.rotation.y)) return true;
                continue;
            }

            // Vật cản thường (không khai báo oob): giữ nguyên cách cũ, AABB-vs-AABB
            this.boxVatCanTemp.setFromObject(obj);
            if (this.boxnhanvat.intersectsBox(this.boxVatCanTemp)) {
                return true;
            }
        }
        return false;
    }

    // CẬP NHẬT VẬT LÝ
    update() {
        const dt = Math.min(this.clock.getDelta(), 0.1); 

        if (!this.status.active) return;

        const danhSachVatCan = this.getAllVatCan();

        // 1. TÍNH HƯỚNG DI CHUYỂN TỪ BÀN PHÍM
        this.huongMuonDi.set(0, 0, 0);

        if (this.status.dichuyen) {
            this.camera.getWorldDirection(this.huongtien);
            this.huongtien.y = 0;
            this.huongtien.normalize();

            this.huongngang.copy(this.trucdung).cross(this.huongtien).normalize();

            const dangChayNhanh = this.status.chay && this.phimnhan["ShiftLeft"];
            const tocdoMucTieu = dangChayNhanh ? this.tocdochaynhanh : this.tocdodichuyen;

            if (this.phimnhan["KeyW"]) this.huongMuonDi.add(this.huongtien);
            if (this.phimnhan["KeyS"]) this.huongMuonDi.sub(this.huongtien);
            if (this.phimnhan["KeyA"]) this.huongMuonDi.add(this.huongngang);
            if (this.phimnhan["KeyD"]) this.huongMuonDi.sub(this.huongngang);

            if (this.huongMuonDi.lengthSq() > 0) {
                this.huongMuonDi.normalize().multiplyScalar(tocdoMucTieu);
            }
        }

        // 2. GIA TỐC & TẠO ĐỘ TRUỢT QUÁN TÍNH
        const lerpFactor = Math.min(this.doMaSat * dt, 1.0);
        this.vanTocHienTai.x += (this.huongMuonDi.x - this.vanTocHienTai.x) * lerpFactor;
        this.vanTocHienTai.z += (this.huongMuonDi.z - this.vanTocHienTai.z) * lerpFactor;

        // 3. XỬ LÝ VA CHẠM TRỤC X & Z (Wall-sliding)
        if (Math.abs(this.vanTocHienTai.x) > 0.001) {
            const deltaX = this.vanTocHienTai.x * dt;
            this.nhanvat.position.x += deltaX;
            if (this.checkvacham()) {
                this.nhanvat.position.x -= deltaX;
                this.vanTocHienTai.x = 0;
            }
        }

        if (Math.abs(this.vanTocHienTai.z) > 0.001) {
            const deltaZ = this.vanTocHienTai.z * dt;
            this.nhanvat.position.z += deltaZ;
            if (this.checkvacham()) {
                this.nhanvat.position.z -= deltaZ;
                this.vanTocHienTai.z = 0;
            }
        }

        // 4. LEO DỐC / MẶT NGHIÊNG
        if (this.status.leodoc && (Math.abs(this.vanTocHienTai.x) > 0.1 || Math.abs(this.vanTocHienTai.z) > 0.1) && danhSachVatCan.length > 0) {
            const viTriBan = this.nhanvat.position.clone().add(new THREE.Vector3(0, 0.5, 0));
            this.raycaster.set(viTriBan, this.huongbanxuong);
            const vaChamDoc = this.raycaster.intersectObjects(danhSachVatCan);

            if (vaChamDoc.length > 0) {
                const doCaoMietDoc = vaChamDoc[0].point.y + (this.kichthuocbox.y / 2);
                const chenhLach = doCaoMietDoc - this.nhanvat.position.y;

                if (chenhLach > 0 && chenhLach < 0.6 && this.trenmatdat) {
                    this.nhanvat.position.y = doCaoMietDoc;
                }
            }
        }

        // 5. XỬ LÝ NHẢY (SPACE)
        if (this.status.nhay && this.phimnhan["Space"] && this.trenmatdat) {
            this.vanTocHienTai.y = this.lucnhay;
            this.trenmatdat = false;
        }

        // 6. TRỌNG LỰC & XỬ LÝ TRỤC Y (Chống chui đất)
        if (this.status.trongluc) {
            this.vanTocHienTai.y -= this.trongluc * dt;
            const deltaY = this.vanTocHienTai.y * dt;
            this.nhanvat.position.y += deltaY;

            if (this.checkvacham()) {
                this.nhanvat.position.y -= deltaY;

                if (this.vanTocHienTai.y < 0) {
                    this.trenmatdat = true;
                }
                this.vanTocHienTai.y = 0;
            } else {
                this.trenmatdat = false;
            }

            
        }
    }
}

// --- CLASS QUẢN LÝ CHÍNH ---
class MCVSystem {
    constructor(camera) {
        this.camera = camera;
        this.vatcan = []; // Vật cản chung toàn hệ thống
        this.controllers = new Map();
    }

    Object3D(nhanvat) {
        if (!this.controllers.has(nhanvat)) {
            const controller = new MotionController(nhanvat, this.camera, this.vatcan);
            this.controllers.set(nhanvat, controller);
        }
        return this.controllers.get(nhanvat);
    }

    motionPhysics() {
        this.controllers.forEach((controller) => {
            controller.update();
        });
    }
}

// Tự động gán vào window để tương thích với HTML script tag thông thường
if (typeof window !== 'undefined') {
    window.MCVSystem = MCVSystem;
}
