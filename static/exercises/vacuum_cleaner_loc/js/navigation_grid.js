var nav_canvas = document.getElementById("nav-grid");
var nav_ctx = nav_canvas.getContext("2d");
var nav_squares = [];
var nav_sizesquare = {w: 10, h: 10};
var nav_count = 0;
var nav_fill_index=[];
var grid_line_width = Math.ceil(nav_sizesquare/35);
var nav_columnas = [];
var nav_filas = [];

function initGrid(rows, cols){
//Drawing nav_canvas grid for first time
    nav_canvas = document.getElementById("nav-grid");
    nav_sizesquare.w = nav_canvas.width/rows;
    nav_sizesquare.h = nav_canvas.height/cols;
    grid_line_width = Math.ceil(nav_sizesquare/35);
    if (nav_canvas && nav_canvas.getContext) {
        nav_ctx = nav_canvas.getContext("2d");
        if (nav_ctx) {
            drawGrid(nav_sizesquare.w, nav_sizesquare.h, grid_line_width, "#44414B", nav_ctx);
        } else {
            alert("No se pudo cargar el contexto");
        }
    }
}

function drawGrid(disX, disY, wLine, color, nav_ctx){
    nav_squares=[];
    nav_ctx.strokeStyle = color;
    nav_ctx.lineWidth = wLine;
    nav_columnas = [];
    nav_filas = [];
    for (i = 0; i < nav_canvas.width; i += disX) {
        nav_ctx.moveTo(i, 0);
        nav_ctx.lineTo(i, nav_canvas.height);
        nav_columnas.push(i);
    }
    for (i = 0; i < nav_canvas.height; i += disY) {
        nav_ctx.moveTo(0, i);
        nav_ctx.lineTo(nav_canvas.width, i);
        nav_filas.push(i);
    }
    nav_ctx.stroke();
    nav_columnas.push(0);
    nav_filas.push(0);

    // Init matrix
    initializeSquaresMatrix();
}

// Funtion that inits the square matrix
function initializeSquaresMatrix(){
    // Empty array
    nav_squares = [];
    // Vars for coords
    let x = 0;
    let y  = 0;
    // Create empty matrix
    for (let i = 0; i < (nav_canvas.height / nav_sizesquare.h) ; i++) {
        nav_squares[i] = [];
        y = 0;
        for (let j = 0; j < (nav_canvas.width / nav_sizesquare.w); j++) {
            nav_squares[i][j] = [x,y,nav_sizesquare.w,nav_sizesquare.h];
            y += nav_sizesquare.w;
        }
        x += nav_sizesquare.h;
    }
}

function fillNavCell(x, y,nav_ctx, color) {
    // Adjust correctly the coordinates flooring
    //x = Math.floor(decimalAdjust('floor', x, 1) / 10);
    //y = Math.floor(decimalAdjust('floor', y, 1) / 10);

    // Set color
    nav_ctx.fillStyle = color;

    if(x >= (nav_canvas.width / nav_sizesquare.w)){
        return;
    }
    if(y >= (nav_canvas.height / nav_sizesquare.h)){
        return;
    }

    // Fill rectangle
    //console.log(nav_squares[y][x][1], nav_squares[y][x][0]);
    nav_ctx.fillRect(nav_squares[y][x][1], nav_squares[y][x][0], nav_sizesquare.w, nav_sizesquare.h);
    // Append node to index list and set the nav_count
    if (!nav_fill_index.includes(nav_squares[y][x])){
        nav_fill_index.push(nav_squares[y][x]);
        nav_count= nav_count + 1;
    }
}



function fillGrid(mat){
    nav_canvas = document.getElementById("nav-grid");
    nav_ctx = nav_canvas.getContext("2d");
	
    let cell_color = '';
    for(let i = 0; i < mat.length; i++) {
        for (let j = 0; j < mat[i].length; j++) {
            if (mat[i][j] == 0)
                cell_color = '#C2C2C2'
            else if (mat[i][j] == 1)
                cell_color = '#287C00';
            else if (mat[i][j] == 2)
                cell_color = '#D6E000';
            else if (mat[i][j] == 3)
                cell_color = '#7C0000';
            fillNavCell(i, j, nav_ctx, cell_color);
        }
    }
    drawGrid(nav_sizesquare.w, nav_sizesquare.h, grid_line_width, "#000000", nav_ctx);
}


function reset_navigation_map() {        
    nav_canvas = document.getElementById("nav-grid");
    nav_ctx = nav_canvas.getContext("2d");
    nav_ctx.beginPath();
    nav_ctx.clearRect(0, 0, nav_canvas.width, nav_canvas.height);
}