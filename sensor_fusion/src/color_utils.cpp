#include "color_utils.hpp"

std::string output_yellow(const std::string& text) {
    return "\033[38;5;214m" + text + "\033[0m";
}

std::string output_green(const std::string& text) {
    return "\033[38;5;46m" + text + "\033[0m";
}

std::string output_red(const std::string& text) {
    return "\033[38;5;196m" + text + "\033[0m";
}

std::string output_blue(const std::string& text) {
    return "\033[38;5;21m" + text + "\033[0m";
}

std::string output_magenta(const std::string& text) {
    return "\033[38;5;201m" + text + "\033[0m";
}

std::string output_cyan(const std::string& text) {
    return "\033[38;5;51m" + text + "\033[0m";
}

std::string output_white(const std::string& text) {
    return "\033[38;5;15m" + text + "\033[0m";
}

std::string output_black(const std::string& text) {
    return "\033[38;5;16m" + text + "\033[0m";
}

std::string output_gray(const std::string& text) {
    return "\033[38;5;8m" + text + "\033[0m";
}