/*
    Requirements:
        - Multidimensional Boolean Array/Vector for each view
        - A view space (all generated views for the corresponding .pcd)
        - This node to handle to formation of view sets using logical xor / or between the multidimensioanl boolean arrays/vectors

    What needs to be made and were:
        - Generate Views
            - Find a way to publish or pass the Views generated in this file tp generateViewSet.cpp
        - Find Points
            - Find a method to either implement in the current findPoints.cpp or create a new .cpp
                in order to create and pass to generateViewSet.cpp a multidimensional boolean
                array/vector
        - Generate View Set
            - Make a method of forming view sets through the use of these boolean sets
                - Using logical OR/XOR we can combine the views and fijnd a way of comparing viewsets

*/
