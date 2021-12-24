function get_inner_size(element) {
    var cs = getComputedStyle(element);
    var padding_x = parseFloat(cs.paddingLeft) + parseFloat(cs.paddingRight);
    var padding_y = parseFloat(cs.paddingTop) + parseFloat(cs.paddingBottom);

    var border_x = parseFloat(cs.borderLeftWidth) + parseFloat(cs.borderRightWidth);
    var border_y = parseFloat(cs.borderTopWidth) + parseFloat(cs.borderBottomWidth);

    // Element width and height minus padding and border
    width = element.offsetWidth - padding_x - border_x;
    height = element.offsetHeight - padding_y - border_y;

    return {width: Math.floor(width), height: Math.floor(height)};
}

function get_novnc_size() {
    var inner_size = get_inner_size(document.querySelector(".split.b"));
    var width = inner_size.width || document.body.clientWidth;
    // Since only 50% of height is used for gazebo iframe
    var height = Math.floor(0.5 * inner_size.height) || document.body.clientHeight;
    return {width: width, height: height};
}


function saveCodeUnibotics(exercise, verbose = true){
// Get the code from editor and add header
console.log("Save code unibotics called");
var python_code = editor.getValue();
var blob = new Blob([python_code], {type: "text/plain; charset=utf-8"});
if (window.navigator.msSaveOrOpenBlob)
  window.navigator.msSaveOrOpenBlob(blob, $("#form29").val());
else{
  console.log("else");
  var a = document.createElement("a"),
  url = URL.createObjectURL(blob);
  a.href = url;
  a.download = exercise + ".py";
  document.body.appendChild(a);
  a.click()
  setTimeout(function(){
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
  }, 0);
  }  
}