/*
 * Package: Think of this like a folder that contains program files. Helps keep your code organized.
 */
package com.example.teachingjava;

/*
 * Class: A class is similar to the blueprint of an object. It has properties and methods that it
 *    can perform.
 * Example: Humans; has properties like Gender, height, and weight; has methods or functions like
 *    walking, running, grabbing something, and speaking.
 *
 * Public: A Java Keyword that allows other files/classes to use this class.
 *
 * LearnJava: The name of the class. Typically, class names start with a capital letter and if the
 *    class name has more than one word each word is capitalized.
 *
 * The code that's part of a class is contained inside a set of curly braces -- { }
 *
 * This file's name is "LearnJava.java" and its main class is called LearnJava. Each file must have
 *    a class named the same as the file but can contain other classes inside of the file.
 */
public class LearnJava_1 {

    /*
     * main: the name of a method.
     *
     * Method: A method, also known as a function, is a section of code that can be used in
     *    multiple places of a class
     *
     * Static: This means you can use this method in other classes without having to instantiate
     *    the class (big word meaning create an actual object from the class blueprint.
     *
     * Return type: Every method has a return type, meaning what the method returns from whatever
     *    it does inside the curly braces. This type is "void," meaning it returns absolutely nada.
     */
    public static void main(String[] args){

        /*
         * Variable: A variable is a specific piece of memory that can hold a value. It's like a
         *    normal variable in Algebra but can have different data types, depending on what kind
         *    information it needs to represent. Typically, a variable name starts with a lowercase
         *    letter.
         *
         * Literal: A literal is specific information. It cannot be changed. Like a specific number
         *    or word.
         *
         * In the statement below, we define a variable named 'variable' and give it the data type
         *    int, or integer. The '=' is called the "assignment operator" and assigns the literal
         *    integer '1' to the variable 'variable.'
         *
         * Each java statement, besides curly braces, ends with a semicolon, which is like a period
         *    in an English sentence.
         */
        int variable = 1;

        /*
         * Data types: Java has several different primitive data types, these are the basis of
         *    most of the properties of a class. The primitive data types are shown below.
         */

        //Byte: integer numbers; ranges from -128 to +127
        byte bite = 11;

        //short: integer numbers; ranges from -32,768 yo +32,767
        short verticallyChallenged = 128;

        //int: integer numbers; ranges from −2,147,483,648 to +2,147,483,647
        int integer = 1270;

        //long: integer numbers; ranges from ... nope, not doing it. You have google.
        long tallBoi = 12709908;

        //float: decimal numbers; 7 digits of accuracy. Must use 'f' or 'F' after the decimal number
        //   to indicate that it is a float.
        float floater = 9.09f;

        //double: decimal number; 15 digits of accuracy.
        double decimal = 7.77;

        //boolean: contains a true or false value.
        boolean facts = true;

        //character: contains characters, or single letters;
        char character = 'a';

        /*
         * You can use arithmetic operations on both literals and variables as long as they are
         *    of the same type. You can store the end result in another variable.
         */

        //Addition
        int addition = 1 + 19;

        //Subtraction
        long subtraction = tallBoi - 180;

        //Multiplication
        double multiply = decimal * integer;

        //Division; gives error if you divide by 0
        float division = 107.9f / floater;

        //Modulus; gives the remainder of division; Example below: result is 3.
        int result = 10 % 7;

        /*
         * You can put data onto the console by using the statement below. You can use literals
         *    of any data type, including String literals.
         */
        System.out.print("This is a String literal. ");
        System.out.println("This print line statement creates a new line after this one.");
        System.out.println(addition);

    }

}
