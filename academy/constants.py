base_py_code = """import WebGUI
import HAL
import Frequency
# Enter sequential code!

while True:
    # Enter iterative code!
    Frequency.tick()
"""

base_cpp_code = """#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"

void exercise() {
    Frequency freq = Frequency();
    // Enter sequential code!

    while (true)
    {
        // Enter iterative code!
        freq.tick();


    }
}
"""