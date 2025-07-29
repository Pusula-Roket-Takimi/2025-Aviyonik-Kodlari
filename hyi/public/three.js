let rocketModel = null;
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
    const camera = new THREE.PerspectiveCamera(45, 2, 0.1, 1000);
    camera.position.set(300, 100, 300); // Daha uzaktan ve yan açıdan
    const controls = new THREE.OrbitControls(camera, canvas);
    controls.target.set(0, 0, 0); // Roketin merkezine odaklan
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
            object.scale.set(3, 3, 3); // Gerekirse bu değeri değiştir
            // Modeli dikey yapmak için başlangıç rotasyonunu ayarla
            object.rotation.y = THREE.MathUtils.degToRad(180); // 0 derecede dikey
            // Otomatik ortalama (bounding box ile)
            const box = new THREE.Box3().setFromObject(object);
            const center = box.getCenter(new THREE.Vector3());
            rocketModel = object; // ⭐ Önemli: Global referans
            scene.add(object);
        });
    });
window.setRocketAngle = function(xAngle,yAngle,zAngle) {
  if (rocketModel) {
     
    rocketModel.rotation.x = THREE.MathUtils.degtoRead(xAngle)
    rocketModel.rotation.y = THREE.MathUtils.degToRad(yAngle);
    rocketModel.rotation.z = THREE.MathUtils.degtoRead(zAngle)
  } else {
    console.warn("Model henüz yüklenmedi.");
  }
};
    function render() {
        if (resizeRendererToDisplaySize(renderer)) {
            const canvas = renderer.domElement;
            camera.aspect = canvas.clientWidth / canvas.clientHeight;
            camera.updateProjectionMatrix();
        }
        renderer.render(scene, camera);
        requestAnimationFrame(render);
    }
    render();
}

window.addEventListener('DOMContentLoaded', main); 
