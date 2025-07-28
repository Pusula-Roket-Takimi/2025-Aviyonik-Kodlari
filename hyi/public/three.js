let rocketModel = null;
function main() {
    const canvas = document.getElementById('3dModelCanvas');
    const renderer = new THREE.WebGLRenderer({ antialias: true, canvas });
    renderer.setClearColor(0x1a1a1a);
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
    camera.position.set(0, 200, 100); // daha uzaktan bak
    const controls = new THREE.OrbitControls(camera, canvas);
    controls.target.set(0, 5, 0);
    controls.update();
    const scene = new THREE.Scene();
    // Işıklar
    scene.add(new THREE.HemisphereLight(0xB1E1FF, 0xB97A20, 2));
    const dirLight = new THREE.DirectionalLight(0xFFFFFF, 2.5);
    dirLight.position.set(0, 10, 0);
    dirLight.target.position.set(-5, 0, 0);
    scene.add(dirLight);
    scene.add(dirLight.target);

    // Model yükle
    const mtlLoader = new THREE.MTLLoader();
    mtlLoader.load('rocket.mtl', function(materials) {
        materials.preload();
        const objLoader = new THREE.OBJLoader();
        objLoader.setMaterials(materials);
        objLoader.load('rocket.obj', function(object) {
            // Modeli merkeze al ve büyüt
            object.scale.set(3, 3, 3); // Gerekirse bu değeri değiştir
            // Otomatik ortalama (bounding box ile)
            const box = new THREE.Box3().setFromObject(object);
            const center = box.getCenter(new THREE.Vector3());
            rocketModel = object; // ⭐ Önemli: Global referans
            scene.add(object);
        });
    });
window.setRocketAngle = function(angle) {
  if (rocketModel) {
        rocketModel.rotation.x = 0
   
            rocketModel.rotation.y = 0;

    rocketModel.rotation.z = THREE.MathUtils.degToRad(angle);
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
