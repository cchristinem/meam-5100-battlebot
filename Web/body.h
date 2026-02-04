/*
 * HTML and Javascript code
 */
const char body[] PROGMEM = R"===(<!DOCTYPE html>
<html>
  <head>
    <title>Robot Control</title>
    <style>
      body {
        font-family: Arial, sans-serif;
        text-align: center;
      }
      .dpad-container {
        display: grid;
        grid-template-columns: 100px 100px 100px; /* Three columns for the D-pad */
        grid-template-rows: 100px 100px 100px;
        gap: 5px;
        margin: 20px auto;
        width: 310px;
      }
      .control-button {
        width: 100%;
        height: 100%;
        font-size: 30px;
        font-weight: bold;
        cursor: pointer;
        border-radius: 5px;
        background-color: #f0f0f0;
        border: 2px solid #ccc;
      }
      .stop-button {
        background-color: #ff5555;
        color: white;
      }
      
      /* Positioning the buttons in the grid */
      #btn-forward { grid-column: 2; grid-row: 1; }
      #btn-left    { grid-column: 1; grid-row: 2; }
      #btn-stop    { grid-column: 2; grid-row: 2; }
      #btn-right   { grid-column: 3; grid-row: 2; }
      #btn-backward{ grid-column: 2; grid-row: 3; }

    </style>
  </head>
  
  <body>
    <h2>Robot Directional Control</h2>
    <div class="dpad-container">
      
      <button 
        class="control-button" 
        id="btn-forward" 
        onmousedown="forward()" 
        onmouseup="stop_release()"
        ontouchstart="forward()" 
        ontouchend="stop_release()">&#9650;</button> 

      <button 
        class="control-button" 
        id="btn-left" 
        onmousedown="left()" 
        onmouseup="stop_release()"
        ontouchstart="left()" 
        ontouchend="stop_release()">&#9664;</button> 

      <button 
        class="control-button stop-button" 
        id="btn-stop" 
        onmousedown="stop_click()" 
        onmouseup="stop_release()"
        ontouchstart="stop_click()" 
        ontouchend="stop_release()">STOP</button> 

      <button 
        class="control-button" 
        id="btn-right" 
        onmousedown="right()" 
        onmouseup="stop_release()"
        ontouchstart="right()" 
        ontouchend="stop_release()">&#9654;</button> 

      <button 
        class="control-button" 
        id="btn-backward" 
        onmousedown="backward()" 
        onmouseup="stop_release()"
        ontouchstart="backward()" 
        ontouchend="stop_release()">&#9660;</button> 
        
    </div>

    <p id="status-message">Press a button to move...</p>
    
    <script>
    // Sends command and updates the SINGLE status message
    function sendCommand(path, message) {
        // **Updates the status message on press or release**
        document.getElementById("status-message").innerHTML = message;
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", path, true);
        xhttp.send();
    }

    // Handlers for button press (onmousedown)
    function forward() { sendCommand("/forward", "Moving Forward"); }
    function backward() { sendCommand("/backward", "Moving Backward"); }
    function left() { sendCommand("/left", "Turning Left"); }
    function right() { sendCommand("/right", "Turning Right"); }

    // New handler for button release (onmouseup)
    function stop_release() {
        // Send the specific command and update the status
        sendCommand("/release_stop", "Released/Stopped");
        // No need to clear individual span IDs anymore!
    }
    
    // Handler for the dedicated Stop button
    function stop_click() {
        // Send the stop command and update the status
        sendCommand("/stop", "Stopped by STOP button");
        // No need to clear individual span IDs anymore!
    }

    </script>
    
    <h2>Autonomous Tasks</h2>
      <h3> Nexus: <\h3>
      <button 
        onmousedown="abort()" 
        onmouseup="stop_release()"
        ontouchstart="abort()" 
        ontouchend="stop_release()">Abort Autonomous Mode</button> <span id="a"></span>
      <button 
        onmousedown="blue_high_tower()" 
        onmouseup="stop_release()"
        ontouchstart="blue_high_tower()" 
        ontouchend="stop_release()">Go to BLUE High Tower</button> <span id="b_h_t"></span>
      <button 
        onmousedown="blue_low_tower()" 
        onmouseup="stop_release()"
        ontouchstart="blue_low_tower()" 
        ontouchend="stop_release()">Go to BLUE Low Tower</button> <span id="b_l_t"></span>
      <button 
        onmousedown="blue_nexus()" 
        onmouseup="stop_release()"
        ontouchstart="blue_nexus()" 
        ontouchend="stop_release()">Go to BLUE Nexus</button> <span id="b_n"></span>
      <button 
        onmousedown="wall()" 
        onmouseup="stop_release()"
        ontouchstart="wall()" 
        ontouchend="stop_release()">Begin Wall Following</button> <span id="w"></span>
      <button 
        onmousedown="ramp()" 
        onmouseup="stop_release()"
        ontouchstart="ramp()" 
        ontouchend="stop_release()">Max Ramp Speed</button> <span id="r"></span>

    <h3>Servo Control</h3>
    <button 
      onmousedown="servo()" 
      onmouseup="stop_release()"
      ontouchstart="servo()" 
      ontouchend="stop_release()">Swing Servo Arm</button> <span id="servoid"></span>
    
    <h3>Vive</h3>
    <form onsubmit="return nav(event)">
      <label for="xcoord">Vive X:</label><br>
      <input type="text" id="xcoord" name="xcoord"><br>
      <label for="ycoord">Vive Y:</label><br>
      <input type="text" id="ycoord" name="ycoord"><br><br>
      <input type="submit" value="Begin Vive Navigation">
    </form> <span id="nav"></span>


    <script>

        function abort() {
          document.getElementById("a").innerHTML = "Abort Autonomous Mode";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/abort", true);
          xhttp.send();
        }
        
        function blue_high_tower() {
          document.getElementById("b_h_t").innerHTML = "Blue High Tower Attack";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/blue_high_tower", true);
          xhttp.send();
        }

        function blue_low_tower() {
          document.getElementById("b_l_t").innerHTML = "Blue Low Tower Attack";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/blue_low_tower", true);
          xhttp.send();
        }

        function blue_nexus() {
          document.getElementById("b_n").innerHTML = "Blue Nexus Attack";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/blue_nexus", true);
          xhttp.send();
        }

        function wall() {
          document.getElementById("w").innerHTML = "Wall Following Mode";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/wall", true);
          xhttp.send();
        }

        function servo() {
          document.getElementById("servoid").innerHTML = "Swing Servo Arm";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/servo", true);
          xhttp.send();
        }

        function ramp() {
          document.getElementById("r").innerHTML = "MAX Ramp Speed";
          var xhttp = new XMLHttpRequest();
          xhttp.open("GET", "/ramp", true);
          xhttp.send();
        }

        function nav(event) {
          event.preventDefault();

          const x = parseInt(document.getElementById("xcoord").value);
          const y = parseInt(document.getElementById("ycoord").value);

          const coords = x * 10000 + y;

          document.getElementById("nav").innerHTML =
            `Vive Navigation Mode: Target (${x}, ${y})`;

          const xhttp = new XMLHttpRequest();
          xhttp.open("GET", `/nav?${coords}`, true);

          xhttp.send();

          return false;
        }

    </script>


    

    
  </body>
</html>
)===";