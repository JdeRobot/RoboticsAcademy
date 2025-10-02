export const defaultPythonCode = `import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
`;

export const defaultCppCode = `#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"
#include "opencv2/opencv.hpp"

void exercise() {
    const Frequency freq = Frequency();

    while (true)
    {
        freq.tick();
        cv::Mat img = HAL::get_image();
        WebGUI::show_image("Hola");
        HAL::set_v(1.0f);
        HAL::set_w(1.0f);
    }
}

`;