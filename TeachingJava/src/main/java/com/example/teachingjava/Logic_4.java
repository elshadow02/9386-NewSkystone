package com.example.teachingjava;

public class Logic_4 {
    /*
     * Java uses logic in its code to determine what code to run. For example, say you write a
     *    program that determines what term best describes a person based solely upon their age. If
     *    someone was less than 13, you would say they are a child. If someone was between 13 and 19,
     *    you would say that they are a teenager. And so on. Java has several ways to accomplish
     *    tasks like this using various statements described below.
     */

    public static void main(String[] args){
        /*
         * The first logic statement is the "If/else" statement. This statement tests one scenario;
         *    if that is true, run the code below it. Otherwise(else), run the other code. Note:
         *    not every "if" statement needs an else to go along with it. In these cases, just
         *    exclude the else statement.
         */

        if(true){
            System.out.println("First statement will run.");
        }
        else{
            System.out.println("First statement will not run.");
        }

        if(false){
            System.out.println("Second statement will not run.");
        }
        else{
            System.out.println("Second statement will run.");
        }

        /*
         * The next logic statement is the "If/else if/else" statement. Similar to the if/else
         *    statement, the if/else-if/else just includes additional tests if the one before it
         *    fails. The following example of the if/else-if/else includes relational operators and
         *    the explanation for them.
         */

        // "==" means "is equal to." Do not confuse the assignment operator, "=,"  which is used to
        //    set a variable, with the equals operator, "==," which is used in logic.
        if (1 == 2){
            System.out.println("Third statement will not run.");
        }
        /*
         * "<" means "less than."
         */
        else if(2 < 1){
            System.out.println("Third statement will not run.");
        }
        // ">" means "greater than."
        else if(1 > 0){
            System.out.println("Third statement will run.");
        }
        // "<=" means "less than or equal to"
        else if(1 <= -9){
            System.out.println("Third statement will not run.");
        }
        // ">=" means "greater than or equal to"
        else if(7 >= 10){
            System.out.println("Third statement will not run.");
        }
        // "!=" means "not equal to"
        else if(10 != 10){
            System.out.println("Third statement will not run.");
        }
        else{
            System.out.println("Third statement will not run.");
        }

        /*
         * Using the "!" before a boolean value reverses the boolean
         */
        if(!true){
            System.out.println("Fourth statement will not run.");
        }
        else if(!false){
            System.out.println("Fourth statement will run.");
        }
        else{
            System.out.println("Fourth statement will not run.");
        }

        /*
         * The next logic statement is the switch statement. It tests a variable against different
         *    cases that it could be equal to. At the very end of the statement, the default case
         *    runs if none of the other cases match the contents of the variable. A break statement
         *    can and usually is included at the end of each case code block. The break statement
         *    tells the program to stop running the code in the switch statement and continue
         *    with the rest of the program. If the break statement is not included, the program
         *    will run the code in the next case code block.
         */

        int placeholder = 2;

        switch(placeholder){
            case 1:
                System.out.println("Switch statement will not run.");
                break;
            case 2:
                System.out.println("Switch statement will run.");
            case 3:
                System.out.println("Switch statement should not run but will run.");
                break;
            case 4:
                System.out.println("Switch statement will not run.");
                break;
            default:
                System.out.println("Switch statement will not run.");
                break;
        }
    }
}
