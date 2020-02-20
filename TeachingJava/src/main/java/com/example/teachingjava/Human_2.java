package com.example.teachingjava;

public class Human_2 {
    /*
     * Variables can be either local or global. Local variables are available to be used only
     *    in the method in which they are declared. Global variables are declared outside of any
     *    method and can be used in any of the classes. Typically, global variables are 'private,'
     *    meaning they can only be accessed inside of their class, in order to make sure nothing bad
     *    happens to them. Global and local variables can have the same name, although, in my opinion,
     *    you should not name them the same. In a case where a local variable or a parameter has the
     *    same name as a global variable, you must use the 'this' keyword, explained later.
     */

    private double height, weight;
    private boolean sarcastic;

    /*
     * String: A String is an object included in the Java Development Kit. Because of this, you can
     *    initialize Strings exactly like variables, unlike user-created classes. Strings hold
     *    String literals, enclosed in double quotations, which can be either a word, letter,
     *    sentence, or even paragraph.
     */
    private String name;

    /*
     * Class initializer: Every class, except for the main class of the program, has to have an
     *    initializer method. The name must be the same as the class and have the public modifier.
     *    This initializer method can have parameters, if you choose to include them.
     */
    public Human_2(String name, double weight, boolean sarcastic){
        //The 'this' keyword references the class it is apart of. You can access global variables
        //   or methods using the 'this' keyword.

        this.name = name;
        this.weight = weight;
        this.sarcastic = sarcastic;
    }

    /*
     * Classes typically have set and get methods that set a global variable or return a global
     *    variable.
     */

    //This method has a parameter. A parameter is some variable, enclosed inside the parenthesis,
    //   that the user inserts the value into when they use the method.
    public void setHeight(double newHeight){
        height = newHeight;
    }

    //This method has a return type of double. It must return a double value.
    public double getHeight(){
        //The return keyword returns the value following it and stops the method.
        return height;
    }

    public String getName(){
        //The return keyword returns the value following it and stops the method.
        return this.name;
    }

    public boolean isSarcastic(){
        return this.sarcastic;
    }

    //Example of a method that is not a get or set
    public void introduce(){
        System.out.println("Hello. My name is " + this.name);
    }
}
