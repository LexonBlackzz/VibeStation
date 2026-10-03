#include "ui/vs2_hosted.h"

#include "ui/ps2_app.h"

namespace ps2::ui {

Vibestation2::Vibestation2() = default;
Vibestation2::~Vibestation2() { shutdown(); }

bool Vibestation2::init(SDL_Window* window, void* gl_context, int gl_major, int gl_minor,
                        const char* glsl, bool opengl2) {
    HostedWindow host;
    host.window = window;
    host.gl_context = gl_context;
    host.gl_major = gl_major;
    host.gl_minor = gl_minor;
    host.glsl = glsl;
    host.opengl2 = opengl2;
    app_ = std::make_unique<Ps2App>();
    if (!app_->init(&host)) {
        app_.reset();
        return false;
    }
    return true;
}

void Vibestation2::activate() {
    if (app_) app_->on_activated();
}

void Vibestation2::deactivate() {
    if (app_) app_->on_deactivated();
}

bool Vibestation2::frame() { return app_ ? app_->frame() : false; }

bool Vibestation2::take_switch_request() { return app_ && app_->take_vs1_switch_request(); }

void Vibestation2::begin_switch_back() {
    if (app_) app_->begin_vs1_switch();
}

void Vibestation2::shutdown() {
    if (app_) {
        app_->shutdown();
        app_.reset();
    }
}

} // namespace ps2::ui
