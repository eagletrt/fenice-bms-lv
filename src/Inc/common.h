#ifndef COMMON_H
#define COMMON_H

/**
 * @brief Returns the size of a static fixed length string
 * @details The input string must be defined on the stack and initialized on the
 *          spot. If the string has been input of sprintf than DO NOT USE THIS
 *          MACRO
 * @param A char[] defined on the stack
 * @return The length of the input string
 */
#define M_STATIC_FIXED_STRING_STRLEN(STATIC_FIXED_STRING_POINTER) \
    (size_t)(sizeof(STATIC_FIXED_STRING_POINTER)                  \
             / sizeof(STATIC_FIXED_STRING_POINTER[0]))
#endif