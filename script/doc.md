# Robot Arm driver documentation

The arm starting in homing mode assuming that it is zeroed, and a user should start out their interaction with the arm, by rezeroing it. Otherwise, the arm might break itself, by moving to a position which it cannot.

The arm can be programmed to perform a sequence of moves. Initially, the sequence is simply moving to its zeroed position.

The following sections explains the different modes of the driver code, and their related commands.

## Homing, :h
This mode enables manual control of the arm, as well as zeroing. Note that the zeroed position for the arm is: 
- Being fully extended
- Being orthogonal to the mounting surface

| command | explaination |
| --- | --- |
| f *int* | Move the first link *x* degrees |
| s *int* | Move the second link *x* degrees |
| w | Print the current recorded angles of the arm, to serial |
| z | Set the current recorded angles of the arm to 0 |

## Program, :p
This mode enables programming a sequence of positions for the arm to follow. When ever the program changes, it will have to be verified, before the arm will run it.

A program consists of a sequence of angles, relative to zero, for each link in the arm. The collection of the *i*'th angle for each sequence is a keyframe, which describes a configuration the arm will take at some point.

As an example the sequence of *f*[0,20,20], will move the first link to its zero, then to angle 20, and the remain there for 1 frame. Both negative and positive angles are supported.

| command | explaination |
| --- | --- |
| f *int** | Set a new sequence for the first link to follow |
| s *int** | Set a new sequence for the second link to follow |
| v | Verify the current sequence |

## Run, :r
This mode executes the sequence of moves described by the programmed sequence. This mode is inaccesible while the programmed sequence is unverified.