let TAU = 6.2831853

// Master arm switch for motors. Safety-critical! Limits motors to no more than 10% RPM, but still...
let MOTORS_ARMED = false

/// Convert radians to degrees
function toDegrees(v) {
    return v * 360. / TAU
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
        headers: {
            "X-CSRFToken": getCookie(),
            "Content-Type": "application/json; charset=UTF-8",
            Accept: "application/json",
            "X-Requested-With": "XMLHttpRequest"
        },
        credentials: "include",
        // body: JSON.stringify(data)
    })
        .then(response => response.json())
        .then(r => {
            const params = r.params
            const controls = r.controls

            // todo: Handle errors; both data connection, and sensor errors
            document.getElementById("pitch-reading").textContent = format(toDegrees(params.s_pitch), 0)
            document.getElementById("roll-reading").textContent = format(toDegrees(params.s_roll), 0)
            document.getElementById("yaw-reading").textContent = format(toDegrees(params.s_yaw), 0)

            document.getElementById("pitch-rate-reading").textContent = format(toDegrees(params.v_pitch), 0)
            document.getElementById("roll-rate-reading").textContent = format(toDegrees(params.v_roll), 0)
            document.getElementById("yaw-rate-reading").textContent = format(toDegrees(params.v_yaw), 0)

            document.getElementById("control-roll-reading").textContent = format(controls.roll, 2)
            document.getElementById("control-pitch-reading").textContent = format(controls.pitch, 2)
            document.getElementById("control-yaw-reading").textContent = format(controls.yaw, 2)
            document.getElementById("control-throttle-reading").textContent = format(controls.throttle, 2)

            document.getElementById("control-arm-reading").textContent = controls.arm_status
            document.getElementById("control-mode-reading").textContent = controls.input_mode

        })
}

function armMotors() {
    // Send a commond to the FC to arm motors.
    fetch("/api/arm_motors", {
        method: "POST",
        headers: {
            "X-CSRFToken": getCookie(),
            "Content-Type": "application/json; charset=UTF-8",
            Accept: "application/json",
            "X-Requested-With": "XMLHttpRequest"
        },
        credentials: "include",
        body: JSON.stringify({arm: "confirm"})
    })
        .then(response => response.json())
        .then(r => {
            console.log("motors armed!")
        })
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