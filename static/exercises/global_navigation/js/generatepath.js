// handle responsively resizing the canvas   
var scale=1.00;
var originalWindowWidth=window.innerWidth;
var originalCanvasWidth=document.getElementById('globalnav-eye').width;
function debounce(func, wait, immediate) {
  var timeout;
  return function() {
    var context = this, args = arguments;
    var later = function() {
      timeout = null;
      if (!immediate) func.apply(context, args);
    };
    var callNow = immediate && !timeout;
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
    if (callNow) func.apply(context, args);
  };
};
var resizeCanvas = debounce(function() {
  scale=window.innerWidth/originalWindowWidth;
  $('#globalnav-eye').css('width',originalCanvasWidth*scale);
  $('#gui-canvas-numpy').css('width',originalCanvasWidth*scale);
}, 250);
window.addEventListener('resize', resizeCanvas);

function resizeImages() {
   scale=window.innerWidth/originalWindowWidth;
   $('#globalnav-eye').css('width',originalCanvasWidth*scale);
   $('#gui-canvas-numpy').css('width',originalCanvasWidth*scale);
 }
resizeImages();


var mapCanvas = document.getElementById("globalnav-eye"),
    ctx = mapCanvas.getContext("2d");


mapCanvas.addEventListener("click",destinationPicker);

var cursorXMap = 0;
var cursorYMap = 0;

// Add event listener to mouse click on the map and get coordinates
function destinationPicker(event){

           cursorX = (event.clientX - mapCanvas.getBoundingClientRect().left)/scale;
           cursorY = (event.clientY - mapCanvas.getBoundingClientRect().top)/scale;
           cursorXMap = cursorX/mapCanvas.width * 400;
           cursorYMap = cursorY/mapCanvas.height * 400;
           return [cursorXMap, cursorYMap];
}

//After receiving array data from the editor,it is used to draw lines(ideal path) on the map
function generatePath(data){
      clearPath();
      drawTargetPosition();
      data = data;
      if (data == null){
         return null
      }
      let minx,miny,maxx,maxy;
      miny = minx = Infinity
      maxx = maxy = -Infinity;
      data.forEach(point => {
            minx = Math.min(minx,point[0]);
            miny = Math.min(miny,point[1]);
            maxx = Math.max(maxx,point[0]);
            maxy = Math.max(maxy,point[1]);
         });
      let rangeX = maxx - minx;
      let rangeY = maxy - miny;
      let range = Math.max(rangeX,rangeY);
      let scale = Math.min(mapCanvas.width,mapCanvas.height);

      for (let i = 0; i < data.length-1; i++) {
         ctx.beginPath();
         ctx.moveTo(data[i][0], data[i][1]);
         ctx.strokeStyle = "#008000";
         ctx.strokeWidth = 100;
            let x = data[i+1][0];
            let y = data[i+1][1];
            //x = ((x-minx) / range) * scale;
            //y = ((y-miny) / range) * scale;
            ctx.lineTo(x,y);
            ctx.stroke();
         }
}

function clearPath() {
   ctx.clearRect(0,0,mapCanvas.width,mapCanvas.height);
}

function drawTargetPosition() {
   if (cursorXMap != 0 || cursorYMap != 0) {
      ctx.beginPath();
      ctx.strokeStyle = "#0000FF";
   
      ctx.moveTo((cursorXMap - 8), (cursorYMap - 8));
      ctx.lineTo((cursorXMap + 8), (cursorYMap + 8));
   
      ctx.moveTo((cursorXMap + 8), (cursorYMap - 8));
      ctx.lineTo((cursorXMap - 8), (cursorYMap + 8));
      ctx.stroke();
      ctx.closePath();
   }
}