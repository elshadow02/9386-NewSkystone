package com.example.teachingjava;

public class ArraysExample_6 {

    public static void main(String[] args){
        /*
         * In Java, arrays are collections of variables, like a list of variables. When you
         *    initialize an array, you must set its size. The example below defines an array of
         *    ints that has a max size of 5
         */
        int[] intArray = new int[5];

        /*
         * Arrays have indexes, the spaces where each separate variable is kept. The first space
         *    is indexed at "0" and the last is indexed at 1 less than the max size. Below is an
         *    example of accessing the variable kept in "intArray" at index 0 and setting that
         *    variable equal to 0.
         */
        intArray[0] = 0;

        if(intArray[0] == 0){
            System.out.println("Index 0 variable: " + intArray[0]);
        }

        /*
         * You can also use a special for loop to step through each of the indexes of an array. The
         *    example below shows two for loops. The first shows the specialized array and the
         *    second shows a regular for loop doing the same thing as the specialized one. The
         *    benefits of using a specialized for loop is that you will never step outside of the
         *    array's bounds, its max size, which would crash your program. The downfall is if you
         *    are setting the values of the array.
         */

        //A for loop to set the values of the array
        for(int i = 0; i < intArray.length; i++){
            intArray[i] = (i + 1);
            System.out.println("Array at index " + i + " is " + intArray[i]);
        }

        //A specialized for loop, called for-each loop, to display the values of the array.
        for(int i : intArray){
            System.out.println("Array value is " + i);
        }

        //For loop to repeat the process of the for-each loop above.
        for(int i = 0; i < intArray.length; i++){
            System.out.println("Array at index " + i + " is " + intArray[i]);
        }
    }
}
