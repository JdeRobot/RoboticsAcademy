var mapCanvas = document.getElementById("birds-eye"),
    ctx = mapCanvas.getContext("2d");

var canvas = document.getElementById("gui_canvas"),
        context = canvas.getContext('2d');
        image = new Image();

mapCanvas.addEventListener("click",destinationPicker);

function destinationPicker(event){
       
           cursorX = event.clientX - mapCanvas.offsetLeft;
           cursorY = event.clientY - mapCanvas.offsetTop ;
           //document.getElementById('x-value').textContent = cursorX;
           //document.getElementById('y-value').textContent = cursorY;
	   ctx.beginPath();
           ctx.strokeStyle = "#0000FF";
           
           ctx.moveTo((cursorX - 8) * 0.66556, (cursorY - 8) * 0.342);
           ctx.lineTo((cursorX + 8) * 0.66556, (cursorY + 8) * 0.342);

           ctx.moveTo((cursorX + 8) * 0.66556, (cursorY - 8) * 0.342);
           ctx.lineTo((cursorX - 8) * 0.66556, (cursorY + 8) * 0.342);
           ctx.stroke();
	   ctx.closePath();
           return [cursorX,cursorY];
	   
}

function generatepath(){
        var arrayImage = JSON.parse(localStorage.getItem("array"));
        //console.log(array)
        //document.getElementById("array").textContent = array;
        image_data = JSON.parse(arrayImage.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;
        if(source != ""){
               image.src = "data:image/png;base64," + source;
	       canvas.width = shape[1];
	       canvas.height = shape[0];
	}
        data = JSON.parse(arrayImage.array);
        
              
        image.onload = function(){
              update_image();
        }

        function update_image(){
	window.requestAnimationFrame(update_image);
	context.drawImage(image, 0, 0);
        }
   
        //document.getElementById("array").value = data;
        var minx,miny,maxx,maxy;
        miny = minx = Infinity
        maxx = maxy = -Infinity;
        data.forEach(dat => {
          dat.forEach(point => {
              minx = Math.min(minx,point[0]);
              miny = Math.min(miny,point[1]);
              maxx = Math.max(maxx,point[0]);
              maxy = Math.max(maxy,point[1]);
           });
        });
       var rangeX = maxx - minx;
       var rangeY = maxy - miny;
       var range = Math.max(rangeX,rangeY);
       var scale = Math.min(mapCanvas.width,mapCanvas.height);
       
       data.forEach(dat => {
          ctx.beginPath();
          ctx.strokeStyle = "#008000";
          
          dat.forEach(point => {
              var x = point[0];
              var y = point[1];
              x = ((x-minx) / range) * scale;
              y = ((y-miny) / range) * scale;
              ctx.lineTo(x,y);
           });
           ctx.stroke();
        });
}

//function hey(event){
//        document.getElementById('x-value').textContent = event.offsetX;
//        document.getElementById('y-value').textContent = event.offsetY;
//}
