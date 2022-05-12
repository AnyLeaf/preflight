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
            // todo: Select units

            // todo: Handle errors; both data connection, and sensor errors
            if (r.pitch.hasOwnProperty('Ok')) {
                document.getElementById("pitch-reading").textContent = format(r.pitch.Ok, 1)
            }

            if (r.roll.hasOwnProperty('Ok')) {
                document.getElementById("roll-reading").textContent = format(r.roll.Ok, 1)
            }

            if (r.yaw.hasOwnProperty('Ok')) {
                document.getElementById("yaw-reading").textContent = format(r.pitch.Ok, 0)
            }

        })


}

function getCookie() {
    let name_ = "csrftoken"
    let cookieValue = null;
    if (document.cookie && document.cookie !== '') {
        var cookies = document.cookie.split(';');
        for (var i = 0; i < cookies.length; i++) {
            var cookie = cookies[i].trim();
            // Does this cookie string begin with the name we want?
            if (cookie.substring(0, name_.length + 1) === (name_ + '=')) {
                cookieValue = decodeURIComponent(cookie.substring(name_.length + 1));
                break;
            }
        }
    }
    return cookieValue;
}