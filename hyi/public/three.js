let rocketModel = null;
let currentRotation = { x: 0, y: 0, z: 0 };

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
    camera.position.set(600, 1500, 600); // Daha yukarıdan ve açılı görüş
    const controls = new THREE.OrbitControls(camera, canvas);
    controls.target.set(0, 200, 0); // Roketin daha yukarısına odaklan
    controls.enableDamping = true; // Yumuşak hareket
    controls.dampingFactor = 0.05;
    controls.maxDistance = 2500; // Maksimum zoom mesafesi
    controls.minDistance = 200; // Minimum zoom mesafesi
    controls.maxPolarAngle = Math.PI * 0.8; // Maksimum açı sınırı (yukarıdan bakış için)
    controls.minPolarAngle = Math.PI * 0.05; // Minimum açı sınırı (aşağıdan bakış için)
    
    // Kameranın başlangıç rotasyonunu ayarla
    camera.lookAt(0, 200, 0);
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
            object.rotation.x = THREE.MathUtils.degToRad(-90); // Dik pozisyon - X ekseni rotasyonu sıfırla
            object.rotation.y = THREE.MathUtils.degToRad(0); // Y ekseni rotasyonu sıfırla
            object.rotation.z = THREE.MathUtils.degToRad(0); // Z ekseni rotasyonu sıfırla
            
            // Roket modelini ekranın ortasına yerleştir
            object.position.set(0, 0, 0);
            
            // Otomatik ortalama (bounding box ile)
            const box = new THREE.Box3().setFromObject(object);
            const center = box.getCenter(new THREE.Vector3());
            object.position.sub(center); // Merkezi sıfırla
            
            // Roketi biraz yukarı taşı
            object.position.y = 50;
            
            rocketModel = object; // ⭐ Önemli: Global referans
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
    if (rocketModel) {
        // Açıları radyana çevir
        const pitchRad = THREE.MathUtils.degToRad(pitch);
        const yawRad = THREE.MathUtils.degToRad(yaw);
        const rollRad = THREE.MathUtils.degToRad(roll);
        
        // Mevcut rotasyonu güncelle - aviyonik sistem dik durduğu için
        currentRotation.x = pitchRad;  // Pitch: Yukarı-aşağı açı
        currentRotation.y = yawRad;    // Yaw: Sağa-sola açı
        currentRotation.z = rollRad;   // Roll: Dönme açısı
        
        // Roket modelini döndür (dik pozisyondan başlayarak)
        // Aviyonik sistem dik durduğu için rotasyonları doğrudan uygula
        rocketModel.rotation.x = currentRotation.x;
        rocketModel.rotation.y = currentRotation.y;
        rocketModel.rotation.z = currentRotation.z;
        
        console.log(`Roket döndürüldü - Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°, Roll: ${roll.toFixed(1)}°`);
    }
};

window.addEventListener('DOMContentLoaded', main); 
