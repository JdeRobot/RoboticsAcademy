export const snippetsBuilder = ({ monaco, range, importName }) => {
  const snippets = [];

  const importSnippets = importSnippetsObj[`_${importName}`];

  if (!importSnippets) return [];

  importSnippets.forEach((snippet) => {
    if (snippet.label && snippet.code) {
      snippets.push({
        label: snippet.label,
        kind:
          snippet.type === "method"
            ? monaco.languages.CompletionItemKind.Method
            : monaco.languages.CompletionItemKind.Variable,
        insertText: snippet.code,
        range: range,
      });
    }
  });

  return snippets;
};

export const extractPythonImports = (code) => {
  const importRegex =
    /\bimport\s+([a-zA-Z_][\w]*)(?:\s+as\s+([a-zA-Z_][\w]*))?/g;
  // const fromImportRegex =
  //     /\bfrom\s+([a-zA-Z_][\w]*)\s+import\s+([\w,*{}]+)(?:\s+as\s+([a-zA-Z_][\w]*))?/g;

  let imports = [];
  let match;

  while ((match = importRegex.exec(code)) !== null) {
    const importName = match[1];
    const alias = match[2] ? match[2] : importName;
    imports.push({ importName, alias });
  }

  // while ((match = fromImportRegex.exec(code)) !== null) {
  //     const importFrom = match[1];
  //     const importWhat = match[2];
  //     const alias = match[3] ? match[3] : null;
  //     imports.push({ importFrom, importWhat, alias });
  // }

  return imports;
};

export const importSnippetsObj = {
  _numpy: [
    {
      type: "method",
      label: "array([])",
      code: "array([])",
      descriptions: "",
    },
    {
      type: "method",
      label: "array([], dtype = float)",
      code: "array([], dtype = float)",
      descriptions: "",
    },
    {
      type: "method",
      label: "asarray([]) ",
      code: "asarray([]) ",
      descriptions: "",
    },
    {
      type: "method",
      label: "fromiter(iterable, float)",
      code: "fromiter(iterable, float)",
      descriptions: "",
    },
    {
      type: "method",
      label: "empty([], dtype=int)",
      code: "empty([], dtype=int)",
      descriptions: "",
    },
    {
      type: "method",
      label: "arange(a, b)",
      code: "arange(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "linspace(a, b, c)",
      code: "linspace(a, b, c)",
      descriptions: "",
    },
    {
      type: "method",
      label: "zeros(a, dtype=int)",
      code: "zeros(a, dtype=int)",
      descriptions: "",
    },
    {
      type: "method",
      label: "ones(a, dtype=int)",
      code: "ones(a, dtype=int)",
      descriptions: "",
    },
    {
      type: "method",
      label: "random.rand(a)",
      code: "random.rand(a)",
      descriptions: "",
    },
    {
      type: "method",
      label: "random.randint(a, size=b)",
      code: "random.randint(a, size=b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "zeros([a, b], dtype = numpy.int32)",
      code: "zeros([a, b], dtype = numpy.int32)",
      descriptions: "",
    },
    {
      type: "method",
      label: "ones([a, b], dtype = numpy.int32)",
      code: "ones([a, b], dtype = numpy.int32)",
      descriptions: "",
    },
    {
      type: "method",
      label: "full([a, b], c, dtype = int)",
      code: "full([a, b], c, dtype = int)",
      descriptions: "",
    },
    {
      type: "method",
      label: "eye(a)",
      code: "eye(a)",
      descriptions: "",
    },
    {
      type: "method",
      label: "genfromtxt('filesmyfile.csv', delimiter=',')",
      code: "genfromtxt('filesmyfile.csv', delimiter=',')",
      descriptions: "",
    },
    {
      type: "method",
      label: "append(arr, [7])",
      code: "append(arr, [7])",
      descriptions: "",
    },
    {
      type: "method",
      label: "arange(a, b).reshape(c, d)",
      code: "arange(a, b).reshape(c, d)",
      descriptions: "",
    },
    {
      type: "method",
      label: "insert(arr, a, b) ",
      code: "insert(arr, a, b) ",
      descriptions: "",
    },
    {
      type: "method",
      label: "delete(arr, object)",
      code: "delete(arr, object)",
      descriptions: "",
    },
    {
      type: "method",
      label: "concatenate((arr1, arr2), axis = 0)",
      code: "concatenate((arr1, arr2), axis = 0)",
      descriptions: "",
    },
    {
      type: "method",
      label: "concatenate((arr1, arr2), axis = 1) ",
      code: " concatenate((arr1, arr2), axis = 1) ",
      descriptions: "",
    },
    {
      type: "method",
      label: "add(a, b)",
      code: "add(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "subtract(a, b)",
      code: "subtract(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "multiply(a, b)",
      code: "multiply(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "divide(a, b)",
      code: "divide(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "mod(a, b)",
      code: "mod(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "remainder(a,b)",
      code: "remainder(a,b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "power(a, b)",
      code: "power(a, b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "exp(b)",
      code: "exp(b)",
      descriptions: "",
    },
    {
      type: "method",
      label: "sqrt(arr)",
      code: "sqrt(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "log(arr)",
      code: "log(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "absolute(arr)",
      code: "absolute(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "sin(arr)",
      code: "sin(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "ceil(arr)",
      code: "ceil(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "floor(arr)",
      code: "floor(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "round_(arr)",
      code: "round_(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "mean(arr)",
      code: "mean(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "median(arr)",
      code: "median(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "sum(arr, dtype = numpy.uint8)",
      code: "sum(arr, dtype = numpy.uint8)",
      descriptions: "",
    },
    {
      type: "method",
      label: "max(arr)",
      code: "max(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "min(arr)",
      code: "min(arr)",
      descriptions: "",
    },
    {
      type: "method",
      label: "var(arr, dtype = numpy.float32)",
      code: "var(arr, dtype = numpy.float32)",
      descriptions: "",
    },
    {
      type: "method",
      label: "std(arr, dtype = numpy.float32)",
      code: "std(arr, dtype = numpy.float32)",
      descriptions: "",
    },
    {
      type: "method",
      label: "corrcoef(array1, array2)",
      code: "corrcoef(array1, array2)",
      descriptions: "",
    },
    {
      type: "method",
      label: "copy(a)",
      code: "copy(a)",
      descriptions: "",
    },
  ],
  _math: [
    {
      type: "method",
      label: "acos()",
      code: "acos()",
      descriptions: "",
    },
    {
      type: "method",
      label: "acosh()",
      code: "acosh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "asin()",
      code: "asin()",
      descriptions: "",
    },
    {
      type: "method",
      label: "asinh()",
      code: "asinh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "atan()",
      code: "atan()",
      descriptions: "",
    },
    {
      type: "method",
      label: "atan2()",
      code: "atan2()",
      descriptions: "",
    },
    {
      type: "method",
      label: "atanh()",
      code: "atanh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "ceil()",
      code: "ceil()",
      descriptions: "",
    },
    {
      type: "method",
      label: "comb()",
      code: "comb()",
      descriptions: "",
    },
    {
      type: "method",
      label: "copysign()",
      code: "copysign()",
      descriptions: "",
    },
    {
      type: "method",
      label: "cos()",
      code: "cos()",
      descriptions: "",
    },
    {
      type: "method",
      label: "cosh()",
      code: "cosh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "degrees()",
      code: "degrees()",
      descriptions: "",
    },
    {
      type: "method",
      label: "dist()",
      code: "dist()",
      descriptions: "",
    },
    {
      type: "method",
      label: "erf()",
      code: "erf()",
      descriptions: "",
    },
    {
      type: "method",
      label: "erfc()",
      code: "erfc()",
      descriptions: "",
    },
    {
      type: "method",
      label: "exp()",
      code: "exp()",
      descriptions: "",
    },
    {
      type: "method",
      label: "expm1()",
      code: "expm1()",
      descriptions: "",
    },
    {
      type: "method",
      label: "fabs()",
      code: "fabs()",
      descriptions: "",
    },
    {
      type: "method",
      label: "factorial()",
      code: "factorial()",
      descriptions: "",
    },
    {
      type: "method",
      label: "floor()",
      code: "floor()",
      descriptions: "",
    },
    {
      type: "method",
      label: "fmod()",
      code: "fmod()",
      descriptions: "",
    },
    {
      type: "method",
      label: "frexp()",
      code: "frexp()",
      descriptions: "",
    },
    {
      type: "method",
      label: "fsum()",
      code: "fsum()",
      descriptions: "",
    },
    {
      type: "method",
      label: "gamma()",
      code: "gamma()",
      descriptions: "",
    },
    {
      type: "method",
      label: "gcd()",
      code: "gcd()",
      descriptions: "",
    },
    {
      type: "method",
      label: "hypot()",
      code: "hypot()",
      descriptions: "",
    },
    {
      type: "method",
      label: "isclose()",
      code: "isclose()",
      descriptions: "",
    },
    {
      type: "method",
      label: "isfinite()",
      code: "isfinite()",
      descriptions: "",
    },
    {
      type: "method",
      label: "isinf()",
      code: "isinf()",
      descriptions: "",
    },
    {
      type: "method",
      label: "isnan()",
      code: "isnan()",
      descriptions: "",
    },
    {
      type: "method",
      label: "isqrt()",
      code: "isqrt()",
      descriptions: "",
    },
    {
      type: "method",
      label: "ldexp()",
      code: "ldexp()",
      descriptions: "",
    },
    {
      type: "method",
      label: "lgamma()",
      code: "lgamma()",
      descriptions: "",
    },
    {
      type: "method",
      label: "log()",
      code: "log()",
      descriptions: "",
    },
    {
      type: "method",
      label: "log10()",
      code: "log10()",
      descriptions: "",
    },
    {
      type: "method",
      label: "log1p()",
      code: "log1p()",
      descriptions: "",
    },
    {
      type: "method",
      label: "log2()",
      code: "log2()",
      descriptions: "",
    },
    {
      type: "method",
      label: "perm()",
      code: "perm()",
      descriptions: "",
    },
    {
      type: "method",
      label: "pow()",
      code: "pow()",
      descriptions: "",
    },
    {
      type: "method",
      label: "prod()",
      code: "prod()",
      descriptions: "",
    },
    {
      type: "method",
      label: "radians()",
      code: "radians()",
      descriptions: "",
    },
    {
      type: "method",
      label: "remainder()",
      code: "remainder()",
      descriptions: "",
    },
    {
      type: "method",
      label: "sin()",
      code: "sin()",
      descriptions: "",
    },
    {
      type: "method",
      label: "sinh()",
      code: "sinh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "sqrt()",
      code: "sqrt()",
      descriptions: "",
    },
    {
      type: "method",
      label: "tan()",
      code: "tan()",
      descriptions: "",
    },
    {
      type: "method",
      label: "tanh()",
      code: "tanh()",
      descriptions: "",
    },
    {
      type: "method",
      label: "trunc()",
      code: "trunc()",
      descriptions: "",
    },
    {
      type: "variable",
      label: "e",
      code: "e",
      descriptions: "",
    },
    {
      type: "variable",
      label: "inf",
      code: "inf",
      descriptions: "",
    },
    {
      type: "variable",
      label: "nan",
      code: "nan",
      descriptions: "",
    },
    {
      type: "variable",
      label: "pi",
      code: "pi",
      descriptions: "",
    },
    {
      type: "variable",
      label: "tau",
      code: "tau",
      descriptions: "",
    },
  ],
  _cv2: [
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_COLOR)",
      code: "imread(image_path, cv2.IMREAD_COLOR)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_GRAYSCALE)",
      code: "imread(image_path, cv2.IMREAD_GRAYSCALE)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imread(image_path, cv2.IMREAD_UNCHANGED)",
      code: "imread(image_path, cv2.IMREAD_UNCHANGED)",
      descriptions: "This method is used to read an image from its path.",
    },
    {
      type: "method",
      label: "imshow(window_name, image)",
      code: "imshow(window_name, image)",
      descriptions: "It is used to show the image in the window.",
    },
    {
      type: "method",
      label: "imwrite(filename, image)",
      code: "imwrite(filename, image)",
      descriptions:
        "This method is used to write or save an image using OpenCV.",
    },
    {
      type: "method",
      label:
        "line(image, start_coordinates, end_coordinates, color_in_bgr, line_thickness)",
      code: "line(image, start_coordinates, end_coordinates, color_in_bgr, line_thickness)",
      descriptions:
        "By using this function we can create a line in the image from start coordinates to end coordinates with a certain thickness which can be mentioned specifically as well.",
    },
    {
      type: "method",
      label:
        "rectangle(image,top_left_vertex_coordinates, lower_right_vertex_coordinates, color_in_bgr, thickness)",
      code: "rectangle(image,top_left_vertex_coordinates, lower_right_vertex_coordinates, color_in_bgr, thickness)",
      descriptions:
        "This function is used to create a box with a certain thickness and color which can be specified as well.",
    },
    {
      type: "method",
      label: "circle(image, center_coordinates, radius, color, thickness)",
      code: "circle(image, center_coordinates, radius, color, thickness)",
      descriptions:
        "It is used to draw a circle whose centre and radius length is given with a certain thickness and the colour of the strokes of the circle.",
    },
    {
      type: "method",
      label: "polylines(image, [pts], isClosed, color, thickness)",
      code: "polylines(image, [pts], isClosed, color, thickness)",
      descriptions:
        "It is used to draw a polygon on any image whose vertex coordinates are provided.",
    },
    {
      type: "method",
      label:
        "putText(image, ‘TextContent’, (‘text_starting_point_coordinates’)",
      code: "putText(image, ‘TextContent’, (‘text_starting_point_coordinates’)",
      descriptions: "",
    },
    {
      type: "method",
      label: "add(image1, image2)",
      code: "add(image1, image2)",
      descriptions: "This function is used to add two images.",
    },
    {
      type: "method",
      label: "subtract(image1, image2)",
      code: "subtract(image1, image2)",
      descriptions: "This function is used to subtract two images.",
    },
    {
      type: "method",
      label: "addWeighted(image1, weight1, image2, weight2, gammaValue)",
      code: "addWeighted(image1, weight1, image2, weight2, gammaValue)",
      descriptions:
        "This is also known as Alpha Blending. This is nothing but a weighted blending process of two images.",
    },
    {
      type: "method",
      label: "bitwise_and(image1, image2, destination, mask)",
      code: "bitwise_and(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise and logical operations between two images.",
    },
    {
      type: "method",
      label: "bitwise_or(image1, image2, destination, mask)",
      code: "bitwise_or(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise or logical operations between two images.",
    },
    {
      type: "method",
      label: "bitwise_not(image, destination, mask)",
      code: "bitwise_not(image, destination, mask)",
      descriptions:
        "This performs bitwise not logical operations between an image and a mask.",
    },
    {
      type: "method",
      label: "bitwise_xor(image1, image2, destination, mask)",
      code: "bitwise_xor(image1, image2, destination, mask)",
      descriptions:
        "This performs bitwise xor logical operations between two images.",
    },
    {
      type: "method",
      label: "inRange(raw_image, lower, upper)",
      code: "inRange(raw_image, lower, upper)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(image, cv2.COLOR_BGR2GRAY)",
      code: "cvtColor(image, cv2.COLOR_BGR2GRAY)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(img, cv2.COLOR_BGR2HSV)",
      code: "cvtColor(img, cv2.COLOR_BGR2HSV)",
      descriptions: "",
    },
    {
      type: "method",
      label: "cvtColor(img, cv2.COLOR_BGR2LAB)",
      code: "cvtColor(img, cv2.COLOR_BGR2LAB)",
      descriptions: "",
    },
    {
      type: "method",
      label: "read()",
      code: "read()",
      descriptions: "",
    },
  ],
};
