let ws; // Variable global para el WebSocket
let reconnectInterval = 1000; // 1 segundo inicial, aumenta progresivamente hasta 30s

function connectWebSocket() {
    ws = new WebSocket(`ws://${window.location.host}/ws`);  // Reemplaza con la IP de tu ESP32

    ws.onopen = () => {
        console.log("Conectado al WebSocket");
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
                let statusElem = document.getElementById("status");
                statusElem.textContent = data.onState ? "Encendido" : "Apagado";
                statusElem.style.color = data.onState ? "green" : "red";
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
}

// Llamar a la función al cargar la página
window.onload = connectWebSocket;

// Enviar el setpoint cuando el usuario lo cambie
document.getElementById("setpoint").addEventListener("change", (event) => {
    const newSetpoint = parseFloat(event.target.value);
    if (!isNaN(newSetpoint)) {
        ws.send(JSON.stringify({ setpoint: newSetpoint }));
    }
});

// Sincronizar el valor del input numérico cuando se mueve el slider
document.getElementById("tempSlider").oninput = function () {
    document.getElementById("setpoint").value = this.value;
};

document.getElementById("tempSlider").addEventListener("change", function () {
    document.getElementById("setpoint").dispatchEvent(new Event("change"));
});

document.getElementById("setpoint").oninput = function () {
    document.getElementById("tempSlider").value = this.value;
};

// Función para alternar el estado de encendido/apagado
function togglePower() {
    let statusElem = document.getElementById("status");
    let Estado = statusElem.innerText.trim();

    let nuevoEstado = (Estado === "Encendido") ? "apagado" : "encendido";
    ws.send(JSON.stringify({ onState: nuevoEstado }));
}