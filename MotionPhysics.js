class MotionController {
    constructor(nhanvat, camera, globalVatCan) {
        this.nhanvat = nhanvat;
        this.camera = camera;
        this.globalVatCan = globalVatCan;

        // --- CẤU HÌNH VẬT LÝ (Tính theo m/s và m/s²) ---
        this.tocdodichuyen = 7.0;       // Tốc độ đi bộ (m/s)
        this.tocdochaynhanh = 13.0;     // Tốc độ chạy nhanh (m/s)
        this.lucnhay = 11.0;            // Vận tốc nhảy đầu (m/s)
        this.trongluc = 28.0;           // Gia tốc trọng lực (m/s²)
        this.doMaSat = 14.0;             // Độ dừng/quán tính (Càng cao dừng càng nhanh)
        
        this.kichthuocbox = new THREE.Vector3(1, 1.8, 1);
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
        this.vanTocHienTai = new THREE.Vector3(); // Vận tốc thực tế (X, Y, Z)
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
        this.boxVatCanTemp = new THREE.Box3(); // Tái sử dụng Box3 tránh ngốn CPU
        this.raycaster = new THREE.Raycaster();

        this.initEvents();
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
            this.clock.start(); // Restart clock khi bật lại
        } else if (this.status.hasOwnProperty(feature)) {
            this.status[feature] = true;
        }
        return this;
    }

    getAllVatCan() {
        return [...this.globalVatCan, ...this.vatcan];
    }

    // Kiểm tra va chạm tối ưu (không new Box3 trong loop)
    checkvacham() {
        const danhSachVatCan = this.getAllVatCan();
        if (danhSachVatCan.length === 0) return false;

        this.boxnhanvat.setFromCenterAndSize(this.nhanvat.position, this.kichthuocbox);
        for (let i = 0; i < danhSachVatCan.length; i++) {
            this.boxVatCanTemp.setFromObject(danhSachVatCan[i]);
            if (this.boxnhanvat.intersectsBox(this.boxVatCanTemp)) {
                return true;
            }
        }
        return false;
    }

    // CẬP NHẬT VẬT LÝ Chuẩn Roblox
    update() {
        const dt = Math.min(this.clock.getDelta(), 0.1); // Giới hạn delta time chống giật lag khi drop FPS

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

        // 2. GIA TỐC & TẠO ĐỘ TRUỢT QUÁN TÍNH (Mượt như Roblox)
        const lerpFactor = Math.min(this.doMaSat * dt, 1.0);
        this.vanTocHienTai.x += (this.huongMuonDi.x - this.vanTocHienTai.x) * lerpFactor;
        this.vanTocHienTai.z += (this.huongMuonDi.z - this.vanTocHienTai.z) * lerpFactor;

        // 3. XỬ LÝ VA CHẠM TRỤC X & Z (Wall-sliding - Trượt tường)
        if (Math.abs(this.vanTocHienTai.x) > 0.001) {
            const deltaX = this.vanTocHienTai.x * dt;
            this.nhanvat.position.x += deltaX;
            if (this.checkvacham()) {
                this.nhanvat.position.x -= deltaX;
                this.vanTocHienTai.x = 0; // Triệt tiêu vận tốc theo trục bị va chạm
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

        // 6. TRỌNG LỰC & XỬ LÝ TRỤC Y
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

            // Reset khi rơi khỏi map
            if (this.nhanvat.position.y < -30) {
                this.nhanvat.position.set(0, 5, 0);
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
