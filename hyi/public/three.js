let rocketModel = null;
let currentRotation = { x: 0, y: 0, z: 0 };
let baseQuaternion = null; // Modelin başlangıç (dik) yönelimi
let calibrationQuaternion = new THREE.Quaternion(); // Kullanıcı kalibrasyonu
let lastEulerForCalib = { roll: 0, pitch: 0, yaw: 0 }; // Son açıları hatırla

function main() {
    const canvas = document.getElementById('3dModelCanvas');
    const renderer = new THREE.WebGLRenderer({ antialias: true, canvas });
    renderer.setClearColor(0x2a2a2a); // Daha nötr koyu gri
    renderer.setPixelRatio(window.devicePixelRatio);
    
    function resizeRendererToDisplaySize(renderer) {
        const canvas = renderer.domElement;
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        const needResize = canvas.width !== width || canvas.height !== height;
        if (needResize) {
            renderer.setSize(width, height, false);
        }
        return needResize;
    }
    
    const camera = new THREE.PerspectiveCamera(60, 2, 0.1, 1000); // FOV'u artır
    camera.position.set(600, 600, 600); // Daha yukarıdan ve açılı görüş
    const controls = new THREE.OrbitControls(camera, canvas);
    controls.target.set(0, 200, 0); // Roketin daha yukarısına odaklan
    controls.enableDamping = true; // Yumuşak hareket
    controls.dampingFactor = 0.05;
    controls.maxDistance = 2500; // Maksimum zoom mesafesi
    controls.minDistance = 200; // Minimum zoom mesafesi
    controls.maxPolarAngle = Math.PI * 0.8; // Maksimum açı sınırı (yukarıdan bakış için)
    controls.minPolarAngle = Math.PI * 0.05; // Minimum açı sınırı (aşağıdan bakış için)
    
    // Kameranın başlangıç rotasyonunu ayarla
    camera.lookAt(0,200, 0);
    controls.update();
    const scene = new THREE.Scene();
    // Işıklar - Daha yumuşak ve doğal
    scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 0.8)); // Daha az şiddetli

    // Model yükle
    const mtlLoader = new THREE.MTLLoader();
    mtlLoader.load('rocket.mtl', function(materials) {
        materials.preload();
        const objLoader = new THREE.OBJLoader();
        objLoader.setMaterials(materials);
        objLoader.load('rocket.obj', function(object) {
            // Modeli merkeze al ve büyüt
            object.scale.set(2, 2, 2); // Boyutu ayarla
            
            // Roket modelini dik yap - aviyonik sistem dik durduğu için
            object.rotation.x = THREE.MathUtils.degToRad(-90);
            object.rotation.y = THREE.MathUtils.degToRad(0);
            object.rotation.z = THREE.MathUtils.degToRad(0);
            
            // Roket modelini ekranın ortasına yerleştir
            object.position.set(0, 0, 0);
            
            // Otomatik ortalama (bounding box ile)
            const box = new THREE.Box3().setFromObject(object);
            const center = box.getCenter(new THREE.Vector3());
            object.position.sub(center); // Merkezi sıfırla
            
            // Roketi biraz yukarı taşı
            object.position.y = 50;
            
            rocketModel = object; // ⭐ Önemli: Global referans
            rocketModel.rotation.order = 'ZYX'; // Havacılık sırası: yaw(Z) → pitch(Y) → roll(X)
            baseQuaternion = rocketModel.quaternion.clone(); // Temel yönelimi kaydet
            scene.add(object);
        });
    });

    function render() {
        if (resizeRendererToDisplaySize(renderer)) {
            const canvas = renderer.domElement;
            camera.aspect = canvas.clientWidth / canvas.clientHeight;
            camera.updateProjectionMatrix();
        }
        
        controls.update(); // Controls'u güncelle
        
        renderer.render(scene, camera);
        requestAnimationFrame(render);
    }
    render();
}

// Roket açısını ayarlama fonksiyonu
window.setRocketAngle = function(pitch, yaw, roll) {
    if (rocketModel && baseQuaternion) {
        // Açıları radyana çevir
        const pitchRad = THREE.MathUtils.degToRad(pitch);
        const yawRad = THREE.MathUtils.degToRad(yaw);
        const rollRad = THREE.MathUtils.degToRad(roll);

        // Havacılık konvansiyonu: yaw(Z), pitch(Y), roll(X)
        // Not: Euler açılarının uygulanma sırası önemlidir.
        const euler = new THREE.Euler(rollRad, pitchRad, yawRad, 'ZYX');
        const q = new THREE.Quaternion().setFromEuler(euler);

        // Başlangıç (dik) yönelimin üzerine uygula
        rocketModel.quaternion.copy(baseQuaternion).multiply(calibrationQuaternion).multiply(q);

        currentRotation.x = rollRad;
        currentRotation.y = pitchRad;
        currentRotation.z = yawRad;
        lastEulerForCalib = { roll: rollRad, pitch: pitchRad, yaw: yawRad };
    }
};

// Klavye ile kalibrasyon: C ile anlık yönelimi nötr yap, R ile sıfırla
window.addEventListener('keydown', (e) => {
    
    if (!rocketModel || !baseQuaternion) return;
    if (e.key === 'c' || e.key === 'C') {
        const qNow = new THREE.Quaternion().setFromEuler(new THREE.Euler(
            lastEulerForCalib.roll,
            lastEulerForCalib.pitch,
            lastEulerForCalib.yaw,
            'ZYX'
        ));
        calibrationQuaternion.copy(qNow).invert();
        console.log('Kalibrasyon uygulandı (C).');
    } else if (e.key === 'r' || e.key === 'R') {
        calibrationQuaternion.identity();
        console.log('Kalibrasyon sıfırlandı (R).');
    }
});

window.addEventListener('DOMContentLoaded', main); 
