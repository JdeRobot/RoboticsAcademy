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

void exercise() {
    Frequency freq = Frequency();
    // Enter sequential code!

    while (true)
    {
        // Enter iterative code!
        freq.tick();


    }
}
`;
