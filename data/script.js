


const ws = new WebSocket(`ws://${window.location.host}/ws`);  // Reemplaza con la IP de tu ESP32
let reconnectInterval = 1000; // 1 segundo inicial, aumentará progresivamente
ws.onopen = () => {
    console.log("Conectado al WebSocket");
    // 🔥 Enviar solicitud de datos al microcontrolador
    const requestData = JSON.stringify({ command: "get_data" });
    ws.send(requestData);
    console.log("Solicitud de datos enviada:", requestData);
    reconnectInterval = 1000; // Reiniciar el intervalo en caso de éxito
    
};

ws.onmessage = (event) => {
    try {
        const data = JSON.parse(event.data);
        if (data.temp !== undefined) {
            document.getElementById("tempValue").textContent = data.temp.toFixed(2);
        }
        if (data.setpoint !== undefined) {
            document.getElementById("setpoint").value = data.setpoint;
            document.getElementById("tempSlider").value = data.setpoint;
        }
        if (data.onState !== undefined) {
            if(data.onState)  {  
                document.getElementById("status").textContent = "Encendido";
                document.getElementById("status").style.color = "green";
            }
            else    {
                document.getElementById("status").textContent = "Apagado";
                document.getElementById("status").style.color = "red";
            }
              
        }
    } catch (error) {
        console.error("Error al procesar JSON:", error);
    }
};

ws.onerror = (error) => {
    console.error("Error en WebSocket:", error);
};

ws.onclose = () => {
    console.log("Conexión perdida. Intentando reconectar...");
    setTimeout(connectWebSocket, reconnectInterval);
    reconnectInterval = Math.min(reconnectInterval * 2, 30000); // Aumenta hasta 30s
};

// Enviar el setpoint cuando el usuario lo cambie
document.getElementById("setpoint").addEventListener("change", (event) => {
    const newSetpoint = parseFloat(event.target.value);
    if (!isNaN(newSetpoint)) {
        const jsonData = JSON.stringify({ setpoint: newSetpoint });
        ws.send(jsonData);
    }
}
);



// Sincronizar el valor del input numérico cuando se mueve el slider
tempSlider.oninput = function () {
    setpoint.value = this.value;
    //setpoint.dispatchEvent(new Event("change")); // 🔥 Disparar el evento "change"
};


// Cuando se suelta el slider, se dispara el evento "change"
tempSlider.addEventListener("change", function () {
    setpoint.dispatchEvent(new Event("change")); // 🔥 Dispara el evento "change"
});

// Sincronizar el valor del slider cuando se cambia el input numérico
setpoint.oninput = function () {
    tempSlider.value = this.value;
    //this.dispatchEvent(new Event("change")); // 🔥 Disparar el evento "change"
};


document.getElementById("botonToggle").addEventListener("click", () => {
    let nuevoEstado = document.getElementById("estadoTexto").innerText === "Encendido" ? "apagado" : "encendido";
    ws.send(JSON.stringify({ onState: nuevoEstado }));  // 🔥 Envía el estado al ESP32
});






