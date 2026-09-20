#include "ui/ps2_app.h"

int main() {
    ps2::ui::Ps2App app;
    if (!app.init()) {
        return 1;
    }

    app.run();
    app.shutdown();
    return 0;
}
