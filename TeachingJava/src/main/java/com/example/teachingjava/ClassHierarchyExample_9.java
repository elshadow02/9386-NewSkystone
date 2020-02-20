package com.example.teachingjava;

public class ClassHierarchyExample_9 {

    //Run this code and play around to learn about class hierarchy.
    public static void main(String[] args){
        Animal_7 monster = new Animal_7(12, false);
        Dog_8 zeke = new Dog_8();
        //A parent class can be initialized as one of its subclasses, but it can only use the parent
        //   class's methods and variables.
        Animal_7 weirdDog = new Dog_8();

        monster.makeSound();
        zeke.bark();
        weirdDog.makeSound();
    }
}
