// let TAU = 6.2831853

// Master arm switch for motors. Safety-critical! Limits motors to no more than 10% RPM, but still...
let MOTORS_ARMED = false

/// Convert radians to degrees
// function toDegrees(v) {
//     return v * 360. / TAU
// }

let HEADERS = {
    "X-CSRFToken": getCookie(),
    "Content-Type": "application/json; charset=UTF-8",
    Accept: "application/json",
    "X-Requested-With": "XMLHttpRequest"
}

function format(val, precision) {
    // Format as a string, rounded and 0-padded
    let result = Math.round(val * 10**precision) / 10**precision

    // Fill to the specified precision
    const dec = result.toString().split('.')[1]
    const len = dec && dec.length > precision ? dec.length : precision
    return Number(result).toFixed(len)
}

function update_readings() {
    // Request readings, and update the display

    fetch("/api/data", {
        method: "GET",
        headers: HEADERS,
        credentials: "include",
        // body: JSON.stringify(data)
    })
        .then(response => response.json())
        .then(r => {
            // todo: Error handling: Display empty readings etc if we have no result from server.


            ATTITUDE_QUAT.w = -r.attitude_quat.w
            ATTITUDE_QUAT.x = r.attitude_quat.x
            ATTITUDE_QUAT.y = r.attitude_quat.y
            ATTITUDE_QUAT.z = r.attitude_quat.z

            let txPwrText
            // See firmware: `ElrsTxPower`.
            switch(r.link_stats.uplink_tx_power) {
                case 1:
                    txPwrText = "10mW"
                    break;
                case 2:
                    txPwrText = "25mW"
                    break;
                case 8:
                    txPwrText = "50mW"
                    break;
                case 3:
                    txPwrText = "100mW"
                    break;
                case 7:
                    txPwrText = "250mW"
                    break;
                default:
                    txPwrText = "(unknown)"
                    break;
            }

            document.getElementById("altimeter-reading").textContent = format(r.altimeter, 0)
            document.getElementById("altimeter-agl-reading").textContent = format(r.altimeter_agl, 0)

            document.getElementById("voltage-reading").textContent = format(r.batt_v, 1)
            document.getElementById("current-reading").textContent = format(r.current, 1)

            document.getElementById("control-roll-reading").textContent = format(r.controls.roll, 2)
            document.getElementById("control-pitch-reading").textContent = format(r.controls.pitch, 2)
            document.getElementById("control-yaw-reading").textContent = format(r.controls.yaw, 2)
            document.getElementById("control-throttle-reading").textContent = format(r.controls.throttle, 2)

            document.getElementById("control-arm-reading").textContent = r.controls.arm_status
            document.getElementById("control-mode-reading").textContent = r.controls.input_mode

            document.getElementById("rssi-1-reading").textContent = "-" + r.link_stats.uplink_rssi_1 + "dB"
            document.getElementById("rssi-2-reading").textContent = "-" + r.link_stats.uplink_rssi_2 + "dB"
            document.getElementById("link-quality-reading").textContent = r.link_stats.uplink_link_quality + "%"
            document.getElementById("snr-reading").textContent = r.link_stats.uplink_snr
            document.getElementById("tx-power-reading").textContent = txPwrText

        })
}

function armMotors() {
    // Send a commond to the FC to arm motors.
    fetch("/api/arm_motors", {
        method: "POST",
        headers: HEADERS,
        credentials: "include",
        body: "confirm",
    })
        .then(response => response.json())
        .then(r => {})
}


function startMotor(motor) {
    fetch("/api/start_motor", {
        method: "POST",
        headers: HEADERS,
        credentials: "include",
        body: motor
    })
        .then(response => response.json())
        .then(r => {})
}

function getCookie() {
    let name_ = "csrftoken"
    let cookieValue = null;
    if (document.cookie && document.cookie !== '') {
        const cookies = document.cookie.split(';');
        for (let i = 0; i < cookies.length; i++) {
            let cookie = cookies[i].trim();
            // Does this cookie string begin with the name we want?
            if (cookie.substring(0, name_.length + 1) === (name_ + '=')) {
                cookieValue = decodeURIComponent(cookie.substring(name_.length + 1));
                break;
            }
        }
    }
    return cookieValue;
}


// Rendering code for WebGL, using Three.js
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 1000 );

const renderer = new THREE.WebGLRenderer({alpha: true});
// renderer.setSize( window.innerWidth, window.innerHeight );
renderer.setSize( 450, 350 );

let container = document.getElementById("attitude-render")
container.appendChild( renderer.domElement );

const geometry = new THREE.BoxGeometry( 1, 1, 0.5 );
// const material = new THREE.MeshBasicMaterial( {color: 0x00ff00} );
// const material = new THREE.MeshDepthMaterial( );
const material = new THREE.MeshMatcapMaterial( );
const cube = new THREE.Mesh( geometry, material );
scene.add( cube );

let ATTITUDE_QUAT = new THREE.Quaternion();

TAU = 2 * Math.PI

camera.position.z = 1.5;
// camera.rotation.x = TAU/2

function animateAttitude() {
    requestAnimationFrame( animateAttitude );

    // console.log("Render")

    cube.rotation.setFromQuaternion(ATTITUDE_QUAT)

    renderer.render( scene, camera );
}
animateAttitude();