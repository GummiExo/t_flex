$(document).ready(function(){
    var WEBSOCKET_ROUTE = "/ws";



    if(window.location.protocol == "http:"){
        //localhost
        var ws = new WebSocket("ws://" + window.location.host + WEBSOCKET_ROUTE);
    }
    else if(window.location.protocol == "https:"){
        //Dataplicity
        var ws = new WebSocket("wss://" + window.location.host + WEBSOCKET_ROUTE);
    }

    ws.onopen = function(evt) {
        $("#ws-status").html("Connected");
        };

    ws.onmessage = function(evt) {
        $("#ws-status").html("Sending Message");
        alert(evt.data)
        };

    ws.onclose = function(evt) {
        $("#ws-status").html("Disconnected");
        };

    // Motor Controllers
    $("#activate_device").click(function(){
        ws.send("activate_device");
        alert('Porfavor espere hasta que el programador tenga luz intermitente')
    });
    // Calibrations
    $("#start_calibration").click(function(){
        var selector = document.getElementById('selector');
        var value = selector[selector.selectedIndex].value;
        if (value == 'angle_cal') {
            ws.send("angle_calibration");
            alert('Iniciando Calibración de Ángulo')
        }
        else if (value == 'stiffness_cal') {
            ws.send("stiffness_calibration");
            alert('Iniciando Calibración de Rigidez')
        }
    });
    $("#info_button_cal").click(function(){
        var selector = document.getElementById('selector');
        var value = selector[selector.selectedIndex].value;
        if (value == 'angle_cal') {
            alert("1.Oprima Iniciar (Bot\u00f3n Verde)\n2.Lleve la articulaci\u00f3n del tobillo a los m\u00e1ximos rangos en dorsiflexi\u00f3n y plantarflexi\u00f3n\n3.Oprima detener (Bot\u00f3n Rojo)");
        }
        else if (value == 'stiffness_cal') {
            alert('1.Oprima Iniciar (Bot\u00f3n Verde)\n2.Lleve los motores hasta el nivel de rigidez deseado\n3.Oprima detener (Bot\u00f3n Rojo)');
        }
        else if (value == 'state'){
            alert('1.Oprimgeventwebsocketa Iniciar (Bot\u00f3n Verde) para ver el estado del dispositivo');
        }
    });
    $("#stop_calibration").click(function(){
        var selector = document.getElementById('selector');
        var value = selector[selector.selectedIndex].value;
        if (value == 'angle_cal') {
            ws.send("stop_angle_calibration");
            alert('Deteniendo Calibración de Ángulo')
        }
        else if (value == 'stiffness_cal') {
            ws.send("stop_stiffness_calibration");
            alert('Deteniendo Calibración de Rigidez')
        }
    });
    // Therapy Mode
    $("#start_therapy").click(function(){
        var repetitions = document.getElementById('repetitions');
        var num_rep = repetitions.value;
        var frequency = document.getElementById('frequency');
        var freq = frequency.value;
        var velocity = document.getElementById('velocity');
        var vel = velocity.value;
        ws.send("start_therapy " + num_rep + " " + freq + " " + vel);
        alert('Iniciando Terapia\nRepeticiones: ' + num_rep + '\nFrecuencia: ' + freq + '\nVelocidad: ' + vel*10 + '%');
    });
    $("#stop_therapy").click(function(){
        ws.send("stop_therapy")
        alert('Deteniendo Terapia')
    });
    // Assistance Mode
    $("#start_assistance").click(function(){
        var patient_name = document.getElementById('patient_name')
        var name = patient_name.value;
        var time_assistance = document.getElementById('time_assistance');
        var time = time_assistance.value;
        var selector_assistance = document.getElementById('selector_assistance');
        var value_assistance = selector_assistance[selector_assistance.selectedIndex].value;
        var selector_algorithm = document.getElementById('selector_algorithm');
        var value_algorithm = selector_algorithm[selector_algorithm.selectedIndex].value;
        if (value_assistance == 'assistance') {
            if (value_algorithm == 'threshold') {
              ws.send("start_assistance " + name + " threshold " + time)
              alert('Iniciando Asistencia para ' + name + '\nTiempo: ' + time + '\nModalidad: Umbrales')
            }
            else if (value_algorithm == 'machine_learning'){
              ws.send("start_assistance " + name + " machine_learning " + time)
              alert('Iniciando Asistencia para ' + name + '\nTiempo: ' + time + '\nModalidad: Machine Learning')
            }
        }
        else if (value_assistance == 'training') {
            ws.send("train_model");
            alert('Iniciando Entrenamiento del Modelo para')
        }
    });
    $("#stop_assistance").click(function(){
        ws.send("stop_assistance")
        alert('Deteniendo Asistencia')
    });
    // BCI
    $("#open_port").click(function(){
        ws.send("open_port");
        alert('Abriendo Puerto');
    });
    $("#start_therapy_bci").click(function(){
        var velocity = document.getElementById('velocity');
        var vel = velocity.value;
        ws.send("start_therapy_bci " + vel);
        alert('Iniciando Terapia\nVelocidad: ' + (vel/15)*100 + '%');
    });
    $("#stop_therapy_bci").click(function(){
        ws.send("stop_therapy_bci")
        alert('Deteniendo Terapia')
    });
    $("#start_assistance_bci").click(function(){
        var velocity = document.getElementById('velocity');
        var vel = velocity.value;
        ws.send("start_assistance_bci " + vel);
        alert('Iniciando Ciclo de Marcha\nVelocidad: ' + (vel/15)*100 + '%');
    });
    $("#stop_assistance_bci").click(function(){
        ws.send("stop_assistance_bci")
        alert('Deteniendo Asistencia')
    });
    //SSH
    $("#open_terminal").click(function(){
        ws.send("open_terminal")
        alert('Abriendo Consola')
    });
    $("#close_terminal").click(function(){
        ws.send("close_terminal")
        alert('Cerrando puertos')
    });
    // Exit and Menu
    $("#menu").click(function(){
        ws.close()
    });
    $("#exit").click(function(){
        alert('Cerrando Procesos Activos');
        ws.send('exit')
    });
    $("#shutdown").click(function(){
        alert('Apagando la Raspberry por favor espere hasta que el botón verde no se encienda');
        ws.send('shutdown')
    });
});
