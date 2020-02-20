package com.example.teachingjava;

public class UseAClassExample_3 {

    public static void main(String[] args){
        /*
         * Declaring a user-created class is similar to declaring a variable, except that after
         *    the assignment operator (=) you must have the 'new' keyword, which creates enough
         *    memory space for a new object, followed by the class's initializer method.
         */
        Human_2 nathaniel = new Human_2("Nathaniel", 116, true);
        Human_2 jaden = new Human_2("Jaden", 128, false);

        /*
         * In order to use a class's internal methods, you must use the specific object's name
         *    followed by the dot (.) followed by the method's name.
         */
        jaden.setHeight(71);
        nathaniel.setHeight(72);

        nathaniel.introduce();
        jaden.introduce();

        System.out.println("Is " + nathaniel.getName() + " sarcastic? is " + nathaniel.isSarcastic());
        System.out.println("Is " + jaden.getName() + " sarcastic? is " + jaden.isSarcastic());
    }
}
