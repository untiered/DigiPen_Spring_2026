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
Project: r.marqueztwisdale_CS350_1.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 01/26/2026
End Header -------------------------------------------------------*/

WARNING: I was unable to build the project in Release mode. It seems like the problem was mainly a mismatch of compiler tools. I believe this is out of my hands.
After doing some research, it seems like the libraries used in the framework were compiled with a different version of the MSVC compiler toolchain.
In order to solve this, I believe we have to recompile the entire project's framework with the most up to date version of Visual Studio 2022.

OPERATION: The project operates as expected for the most part. There are two parts of the assignment that I had to mess with.
	1. I had to replace 81 instances of "%d" with "%zd" in the testing file for assignment 2. (I was using this to test the debug drawing operations for extra credit)
	2. I had to use the "AttemptNormalize()" function in the Plane::Set functions within the file: Shapes.cpp. This is because the application kept crashing at
	   seemingly random moments when I would click within the window. The crash was caused by division of 0 when attempting to normalize the plane's normal vector.

DEBUG DRAW: I implemented all debugDraw operations.
