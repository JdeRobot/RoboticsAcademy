var canvas = document.getElementById("grid");
var squares = [];
var sizeSquare = {w: 10, h: 10};
var color = "";
var inputColor = document.getElementById("color");
var score = document.getElementById("score");
var count = 0;
var fill_index=[];
var grid_line_width = Math.ceil(sizeSquare/35);
var columnas = [];
var filas = [];

function initGrid(rows, cols){
//Drawing canvas grid for first time
    var canvas = document.getElementById("grid");
    sizeSquare.w = canvas.width/rows;
    sizeSquare.h = canvas.height/cols;
    grid_line_width = Math.ceil(sizeSquare/35);
    if (canvas && canvas.getContext) {
        var ctx = canvas.getContext("2d");
        if (ctx) {
            dibujaGrid(sizeSquare.w, sizeSquare.h, grid_line_width, "#44414B", ctx);
        } else {
            alert("No se pudo cargar el contexto");
        }
    }
}

function dibujaGrid(disX, disY, wLine, color, ctx){
    squares=[];
    ctx.strokeStyle = color;
    ctx.lineWidth = wLine;
    columnas = [];
    filas = [];
    for (i = 0; i < canvas.width; i += disX) {
        ctx.moveTo(i, 0);
        ctx.lineTo(i, canvas.height);
        columnas.push(i);
    }
    for (i = 0; i < canvas.height; i += disY) {
        ctx.moveTo(0, i);
        ctx.lineTo(ctx.canvas.width, i);
        filas.push(i);
    }
    ctx.stroke();
    columnas.push(0);
    filas.push(0);

    // Init matrix
    initSquaresMatrix();
}

// Function user for adjusting the coordinates
// https://developer.mozilla.org/es/docs/Web/JavaScript/Reference/Global_Objects/Math/floor 
function decimalAdjust(type, value, exp) {
    // Si el exp es indefinido o cero...
    if (typeof exp === 'undefined' || +exp === 0) {
        return Math[type](value);
    }
    value = +value;
    exp = +exp;
    // Si el valor no es un número o el exp no es un entero...
    if (isNaN(value) || !(typeof exp === 'number' && exp % 1 === 0)) {
        return NaN;
    }
    // Cambio
    value = value.toString().split('e');
    value = Math[type](+(value[0] + 'e' + (value[1] ? (+value[1] - exp) : -exp)));
    // Volver a cambiar
    value = value.toString().split('e');
    return +(value[0] + 'e' + (value[1] ? (+value[1] + exp) : exp));
}

// Funtion that inits the square matrix
function initSquaresMatrix(){
    // Empty array
    squares = [];
    // Vars for coords
    let x = 0;
    let y  = 0;
    // Create empty matrix
    for (let i = 0; i < (canvas.height / sizeSquare.h) ; i++) {
        squares[i] = [];
        y = 0;
        for (let j = 0; j < (canvas.width / sizeSquare.w); j++) {
            squares[i][j] = [x,y,sizeSquare.w,sizeSquare.h];
            y += sizeSquare.w;
        }
        x += sizeSquare.h;
    }
}

function fillCell(x, y,ctx, color) {
    // Adjust correctly the coordinates flooring
    //x = Math.floor(decimalAdjust('floor', x, 1) / 10);
    //y = Math.floor(decimalAdjust('floor', y, 1) / 10);

    // Set color
    ctx.fillStyle = color;

    if(x >= (canvas.width / sizeSquare.w)){
        return;
    }
    if(y >= (canvas.height / sizeSquare.h)){
        return;
    }

    // Fill rectangle
    //console.log(squares[y][x][1], squares[y][x][0]);
    ctx.fillRect(squares[y][x][1], squares[y][x][0], sizeSquare.w, sizeSquare.h);
    // Append node to index list and set the count
    if (!fill_index.includes(squares[y][x])){
        fill_index.push(squares[y][x]);
        count= count + 1;
    }
}



function fillGrid(mat){
    var canvas = document.getElementById("grid");
    var ctx = canvas.getContext("2d");
	
    let cell_color = '';
    for(let i = 0; i < mat.length; i++) {
        for (let j = 0; j < mat[i].length; j++) {
            if (mat[i][j] == 0)
                cell_color = '#C2C2C2'
            else if (mat[i][j] == 1)
                cell_color = '#287C00';
            else if (mat[i][j] == 2)
                cell_color = '#7C0000';
            else if (mat[i][j] == 3)
                cell_color = '#D6E000';
            fillCell(i, j, ctx, cell_color);
        }
    }
    dibujaGrid(sizeSquare.w, sizeSquare.h, grid_line_width, "#000000", ctx);
}


function reset_evaluator_map() {        
    var canvas = document.getElementById("grid");
    var ctx = canvas.getContext("2d");
    ctx.beginPath();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
}