var mapCanvas = document.getElementById("birds-eye"),
    ctx = mapCanvas.getContext("2d");


mapCanvas.addEventListener("click",destinationPicker);

// Add event listener to mouse click on the map and get coordinates
function destinationPicker(event){
       
           cursorX = event.clientX - mapCanvas.offsetLeft;
           cursorY = event.clientY - mapCanvas.offsetTop ;
   
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

//After receiving array data from the editor,it is used to draw lines(ideal path) on the map
function generatePath(data){
        data = data;
        if (data == null){
           return null
        }
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


