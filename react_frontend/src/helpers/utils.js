export const get_inner_size = (element) => {
  const cs = getComputedStyle(element);
  const padding_x = parseFloat(cs.paddingLeft) + parseFloat(cs.paddingRight);
  const padding_y = parseFloat(cs.paddingTop) + parseFloat(cs.paddingBottom);

  const border_x =
    parseFloat(cs.borderLeftWidth) + parseFloat(cs.borderRightWidth);
  const border_y =
    parseFloat(cs.borderTopWidth) + parseFloat(cs.borderBottomWidth);

  // Element width and height minus padding and border
  const width = element.offsetWidth - padding_x - border_x;
  const height = element.offsetHeight - padding_y - border_y;

  return { width: Math.floor(width), height: Math.floor(height) };
};

export const get_novnc_size = () => {
  const inner_size = get_inner_size(document.querySelector(".split.b"));
  const width = inner_size.width || document.body.clientWidth;
  // Since only 50% of height is used for gazebo iframe
  const height =
    Math.floor(0.5 * inner_size.height) || document.body.clientHeight;
  return { width: width, height: height };
};

export const saveCode = (fileName, python_code) => {
  // Get the code from editor and add header
  console.log("Save Code ");
  const blob = new Blob([python_code], { type: "text/plain; charset=utf-8" });
  const a = document.createElement("a"),
    url = URL.createObjectURL(blob);
  a.href = url;
  a.download = fileName + ".py";
  document.body.appendChild(a);
  a.click();
  setTimeout(function () {
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
  }, 0);
};
