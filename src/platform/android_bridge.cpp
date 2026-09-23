#include "platform/android_bridge.h"

#if defined(__ANDROID__)

#include <SDL_system.h>
#include <jni.h>
#include <mutex>
#include <utility>

namespace {
std::mutex g_picker_mutex;
bool g_picker_has_result = false;
AndroidPickerResult g_picker_result{};

std::string from_jstring(JNIEnv* env, jstring value) {
    if (value == nullptr) {
        return {};
    }
    const char* chars = env->GetStringUTFChars(value, nullptr);
    if (chars == nullptr) {
        return {};
    }
    std::string result(chars);
    env->ReleaseStringUTFChars(value, chars);
    return result;
}
}

bool android_request_picker(AndroidPickerKind kind) {
    JNIEnv* env = static_cast<JNIEnv*>(SDL_AndroidGetJNIEnv());
    jobject activity = SDL_AndroidGetActivity();
    if (env == nullptr || activity == nullptr) {
        return false;
    }

    jclass activity_class = env->GetObjectClass(activity);
    if (activity_class == nullptr) {
        env->DeleteLocalRef(activity);
        return false;
    }

    jmethodID method = env->GetMethodID(
        activity_class, "requestVibeStationPicker", "(I)V");
    if (method == nullptr) {
        env->DeleteLocalRef(activity_class);
        env->DeleteLocalRef(activity);
        return false;
    }

    env->CallVoidMethod(activity, method, static_cast<jint>(kind));
    const bool ok = !env->ExceptionCheck();
    if (!ok) {
        env->ExceptionClear();
    }

    env->DeleteLocalRef(activity_class);
    env->DeleteLocalRef(activity);
    return ok;
}

bool android_poll_picker_result(AndroidPickerResult& out) {
    std::lock_guard<std::mutex> lock(g_picker_mutex);
    if (!g_picker_has_result) {
        return false;
    }
    out = std::move(g_picker_result);
    g_picker_result = {};
    g_picker_has_result = false;
    return true;
}

extern "C" JNIEXPORT void JNICALL
Java_com_lexonblackzz_vibestation_VibeStationActivity_nativeOnPickerResult(
    JNIEnv* env, jclass, jint kind, jboolean cancelled, jstring path,
    jstring display_name) {
    AndroidPickerResult result{};
    result.kind = static_cast<AndroidPickerKind>(kind);
    result.cancelled = cancelled == JNI_TRUE;
    result.path = from_jstring(env, path);
    result.display_name = from_jstring(env, display_name);

    std::lock_guard<std::mutex> lock(g_picker_mutex);
    g_picker_result = std::move(result);
    g_picker_has_result = true;
}

#else

bool android_request_picker(AndroidPickerKind) {
    return false;
}

bool android_poll_picker_result(AndroidPickerResult&) {
    return false;
}

#endif
