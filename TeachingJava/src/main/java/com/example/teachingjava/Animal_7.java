package com.example.teachingjava;

/*
 * A class used in the Dog class to explain class hierarchy.
 */
public class Animal_7 {
    //Protected keyword is an access-modifier that only allows a class's subclasses or any class in
    //    the same package to access that variable or method.
    protected int numberoOfLegs;
    protected boolean loving;

    //Used as a placeholder to explain private vs. protected access.
    private double placeholder = 7;

    public Animal_7(int numberOfLegs, boolean loving){
        this.numberoOfLegs = numberOfLegs;
        this.loving = loving;
    }

    protected void makeSound(){
        System.out.println("Sound placeholder");
    }
}
