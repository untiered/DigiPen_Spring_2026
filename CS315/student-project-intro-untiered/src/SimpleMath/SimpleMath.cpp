/// @author Roman Marquez-Twisdale
/// @date 01/10/2026
/// @par Description: A simple math library providing basic arithmatic functionality.

#include "include/SimpleMath.h"

/// @brief calculate the sum of two integers
/// @param a an integer value
/// @param b an integer value
/// @return the sum of a and b
int simple_add(int a, int b)
{
	return a+b;
}

/// @brief subtract one integer from another
/// @param a an integer representing the minuend of this subtraction
/// @param b an integer representing the subtrahend of this subtraction
/// @return the integer result of subtracting b from a
int simple_sub(int a, int b)
{
	return a-b;
}

/// @brief calculate the product of two integers
/// @param a an integer value
/// @param b an integer value
/// @return the product of a and b
int simple_mul(int a, int b)
{
	return a*b;
}

/// @brief calculate the quotient of two integers
/// @param a an integer representing the dividend of this devision
/// @param b an integer representing the divisor of this division
/// @return the integer result of a divided by b
int simple_div(int a, int b)
{
	return a/b;
}
