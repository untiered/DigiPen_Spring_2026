/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: README.txt
Purpose: This file contains information on how to run the
program, how to use parts of the user interface not specified in the assignment description,
and any assumptions about how to interact with the program that, if violated, might cause
the program to fail. Other useful information that this file should indicate include the
hardware, operating system, settings, etc., that the student compiled, linked, and tested the
program in.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_4.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 04/08/2026
End Header -------------------------------------------------------*/

OPERATION: The project operates as expected for the most part. There is one part of the assignment that fails.
	- the construct function fails when tested on a sphere.
A bunch of my friends got the same exact error. I think it is the result of a floating point operation rounding incorrectly.