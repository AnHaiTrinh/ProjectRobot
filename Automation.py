from QuadTree import main

scenario = input("Enter scenario: ")
algorithm = input("Enter algorithm: ")
for i in range(1, 11):
    main(algorithm, scenario, scenario + str(i), False)
