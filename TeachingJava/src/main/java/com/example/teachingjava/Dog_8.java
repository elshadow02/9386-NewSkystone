package com.example.teachingjava;

/*
 * Classes have a hierarchy. Classes can have children that extend their variables or methods. The
 *    "super" keyword accesses the parent's content like the "this" keyword accesses a class's own
 *    variables and methods.
 *
 * Play around with this subclass to find out more about class hierarchy.
 */
public class Dog_8 extends Animal_7 {

    public Dog_8(){
        /*
         * The "super" keyword here accesses the parent class's initializer method.
         */
        super(4, true);
    }

    /*
     * A child class can override its parent's methods. These methods can have the same name and
     *    return type but different code. If you want to use the parent class's original function
     *    code with stuff added onto it, you could use super.(whateverMethodName).
     */
    @Override
    protected void makeSound() {
        System.out.println("Woof!");
    }

    public void bark(){
        makeSound();
    }
}
