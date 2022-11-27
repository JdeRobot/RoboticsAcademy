import '../style.css'
import * as THREE from 'three'

function createLine(a, b, color = 0xff00ff) {
    const material = new THREE.LineBasicMaterial({ color: color });

    const points = [];
    points.push(new THREE.Vector3(a[0], a[1], a[2]));
    points.push(new THREE.Vector3(b[0], b[1], b[2]));

    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geometry, material);

    return line;
}

export function createFrame(width = 16, height = 9, scale = 1, color = 0xff0000) {
    const frame = new THREE.Group();

    // set camare center
    const center0 = [0, 0, 0]

    // scales
    scale *= 10;
    const distance = .4 * scale
    height = (height / width) * scale
    width = scale

    // frame points
    const pointA = [distance, height / 2, -width / 2]
    const pointB = [distance, height / 2, width / 2]
    const pointC = [distance, -height / 2, width / 2]
    const pointD = [distance, -height / 2, -width / 2]

    const points = [pointA, pointB, pointC, pointD]

    // draw lines
    // center to corner
    for (let i = 0; i < points.length; ++i) {
        frame.add(createLine(center0, points[i]))
    }

    // corner to corner
    for (let i = 1; i < points.length; ++i) {
        frame.add(createLine(points[i - 1], points[i], color))
    }
    frame.add(createLine(points[3], points[0], color))

    // x edge guide
    frame.add(createLine(center0, [distance * 4, 0, 0], 0xBDB76B))

    return frame;
}
