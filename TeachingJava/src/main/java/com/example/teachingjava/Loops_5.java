package com.example.teachingjava;

public class Loops_5 {

    public static void main(String[] args){
        /*
         * Sometimes, you want to repeat code if a certain condition is still true. In these cases,
         *    you want to use loops.
         */

        /*
         * The "while" loop repeats the code contained in its curly braces while a certain condition
         *    remains true. Be careful when using loops. If the condition it tests will never
         *    change to false, the loop becomes an inifite loop and will crash your program.
         */
        int counter = 0;

        while(counter < 5){
            System.out.println("Loop has run " + counter + " times.");

            //A break statement "breaks" the loop and stops it prematurely. Here, the break
            //   statement won't run.
            if(false){
                break;
            }

            /*
             * This statement introduces the "increment" operator, "++". It replaces a line of code like
             *    "counter = counter + 1," or "counter += 1." Basically, it increases the variable
             *    by 1. The decrement operator, "--," does the reverse. It decreases the value of
             *    the variable by 1.
             */
            counter++;
        }

        /*
         * Another loop is similar to the while loop but always runs its code block at least once.
         *    The "do-while" loop runs the code in the "do" block before testing the condition.
         */
        counter = 5;
        do{
            System.out.println("Loop has " + counter + " runs left.");

            /*
             * An example of the decrement operator.
             */
            counter--;
        } while(counter > 0);

        /*
         * The "for" loop runs through its loop in increments. You define a variable, or use a
         *    pre-defined one, test for the condition, and increment or decrement the variable
         *    all in one line. Typically, for loops are used when you know how many times you want
         *    it to loop.
         */
        for(int i = 0; i < 4; i++){
            System.out.println("For loop has run" + counter + " times.");
        }

        counter = 5;

        for( ; counter > 0; counter--){
            System.out.println("For loop has " + counter + " runs left.");
        }

        /*
         * Loops can test for multiple conditions with logical operators, AND(&&) and OR(||).
         */
        counter = 0;
        int secondCounter = 5;

        //Runs only if both conditions are met.
        while(counter < 5 && secondCounter > 0){
            System.out.println("Loop has " + counter + " runs left.");

            counter++;
            secondCounter--;
        }

        counter = 0;
        secondCounter = 5;

        //Runs if one of the conditions is met.
        while(counter < 5 || secondCounter > 0){
            System.out.println("Loop has " + counter + " runs left.");

            counter++;
            secondCounter--;
        }
    }
}
