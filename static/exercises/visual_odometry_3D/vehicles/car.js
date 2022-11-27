import '../style.css'
import * as THREE from 'three'

function createWheels(scale) {
    const geometry = new THREE.BoxBufferGeometry(12 * scale, 12 * scale, 33 * scale);
    const material = new THREE.MeshLambertMaterial({ color: 0x333333 });
    const wheel = new THREE.Mesh(geometry, material);
    return wheel;
}

export function createCar(scale = 1) {
    const car = new THREE.Group();

    const backWheel = createWheels(scale);
    backWheel.position.y = 6 * scale;
    backWheel.position.x = -18 * scale;
    car.add(backWheel);

    const frontWheel = createWheels(scale);
    frontWheel.position.y = 6 * scale;
    frontWheel.position.x = 18 * scale;
    car.add(frontWheel);

    const main = new THREE.Mesh(
        new THREE.BoxBufferGeometry(60 * scale, 15 * scale, 30 * scale),
        new THREE.MeshLambertMaterial({ color: 0x78b14b })
    );
    main.position.y = 12 * scale;
    car.add(main);

    const cabin = new THREE.Mesh(
        new THREE.BoxBufferGeometry(33 * scale, 12 * scale, 24 * scale),
        new THREE.MeshLambertMaterial({ color: 0xffffff })
    );
    cabin.position.x = -6 * scale;
    cabin.position.y = 25.5 * scale;
    car.add(cabin);

    return car;
}