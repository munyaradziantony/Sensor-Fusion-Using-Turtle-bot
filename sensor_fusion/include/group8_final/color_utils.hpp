/**
 * @file color_utils.hpp
 * @author Zeid Kootbally (zeidk@umd.edu)
 * 
 * @brief Provides methods for colorized logging.
 * @version 0.1
 * @date 2024-12-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <string>

/**
 * @brief Formats the given text with yellow color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in yellow color.
 */
std::string output_yellow(const std::string& text);

/**
 * @brief Formats the given text with green color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in green color.
 */
std::string output_green(const std::string& text);

/**
 * @brief Formats the given text with red color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in red color.
 */
std::string output_red(const std::string& text);

/**
 * @brief Formats the given text with blue color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in blue color.
 */
std::string output_blue(const std::string& text);

/**
 * @brief Formats the given text with magenta color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in magenta color.
 */
std::string output_magenta(const std::string& text);

/**
 * @brief Formats the given text with cyan color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in cyan color.
 */
std::string output_cyan(const std::string& text);

/**
 * @brief Formats the given text with white color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in white color.
 */
std::string output_white(const std::string& text);

/**
 * @brief Formats the given text with black color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in black color.
 */
std::string output_black(const std::string& text);

/**
 * @brief Formats the given text with gray color using ANSI escape codes.
 *
 * @param text The input text to be formatted.
 * @return A string formatted in gray color.
 */
std::string output_gray(const std::string& text);