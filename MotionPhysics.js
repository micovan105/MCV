/**
 * MotionPhysics.js - MCV System
 * Bộ quản lý di chuyển & vật lý đơn giản cho Three.js (Fixed Collision & Dynamic BoundingBox)
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
        this.offsetBox = new THREE.Vector3(0, 0, 0); // Độ lệch tâm
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
        
        this.boxnhanvat = new THREE.Box3();
        this.boxVatCanTemp = new THREE.Box3(); 
        this.raycaster = new THREE.Raycaster();

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

    // Kiểm tra va chạm đã sửa logic ôm trọn nhân vật
    checkvacham() {
        const danhSachVatCan = this.getAllVatCan();
        if (danhSachVatCan.length === 0) return false;

        // Cập nhật vị trí Box3 chuẩn xác dựa trên vị trí nhân vật
        this.boxnhanvat.setFromCenterAndSize(this.nhanvat.position, this.kichthuocbox);

        for (let i = 0; i < danhSachVatCan.length; i++) {
            this.boxVatCanTemp.setFromObject(danhSachVatCan[i]);
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

            // Reset khi rơi khỏi map (Đưa về vị trí safe)
            if (this.nhanvat.position.y < -50) {
                this.nhanvat.position.set(0, 20, 0);
                this.vanTocHienTai.set(0, 0, 0);
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
