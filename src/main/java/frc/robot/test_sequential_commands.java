// First task is to get a single button that will move the elevator THEN move the end effector
// We'll need to make a sequence of commands instead of a  single command per button

//go to robot container and the configure button bindings section
//try binding a button to run a "Commands.sequence(thing1, thing2)"
// if you want more details go here

// https://docs.wpilib.org/en/2020/docs/software/commandbased/command-groups.html
// https://docs.wsr.studica.com/en/latest/docs/Software/programming/autonomous/auto-basics/command-groups/sequential-command-group.html

//Expected end product: elevator moves to L4 THEN scores to the right side


//Task 2:  bind another button to reset the whole system
//a little more complicated this time.  But let's have it bring the elevator down and the end effector back at the same time
//this is a parallel command

//commands.sequence(commands.parallel(thing1, thing2))

//Expected end product: elevator and end effector move back to the default position at the press of a single button


//Task 3: combine both tasks into a single button
//Expected end product: go up with elevator, then end effector, then (reset(endeffector, elevator) simultaniously)