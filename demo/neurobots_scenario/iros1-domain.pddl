(define (domain neurobots-iros1)
  (:requirements :adl :object-fluents)
  (:types
   furniture human - location
   location robot - objectlocation
   table - furniture
   bottle cup - vessel
   content
   )

  (:constants
   empty - content
   nowhere - location
   )

  (:predicates
   (is-open ?v - vessel)
   (arm-empty ?r - robot)
   (mobile ?r - robot) ;; only mobile robots can move around
   ;;(has-drunk ?h - human)
   )

  (:functions
   (position ?v - vessel) - objectlocation
   (at ?r - robot) - location
   (contains ?v - vessel) - content
  )

  (:action grasp
           :parameters (?rob - robot ?v - vessel ?b - furniture)
           :precondition (and (= (at ?rob) ?b)
                              (= (position ?v) ?b)
                              (arm-empty ?rob)
                              )
           :effect (and (assign (position ?v) ?rob)
                        (not (arm-empty ?rob))))


  (:action approach
           :parameters (?rb - robot ?bs ?oldlocation - location)
           :precondition (and (= (at ?rb) ?oldlocation)	                             
                              )
           :effect (and (assign (at ?rb) ?bs)))

  (:action drop
           :parameters (?dropper - robot ?v - vessel ?fu - furniture)
           :precondition (and (= (position ?v) ?dropper)
                              (= (at ?dropper) ?fu)
                              )
           :effect (and (assign (position ?v) ?fu)
                        (arm-empty ?dropper)))

  (:action pour
           :parameters (?rob - robot ?o1 ?o2 - vessel ?f - table ?c - content )
           :precondition (and (= (position ?o1) ?rob)
                              (= (position ?o2) ?f)
                              (= (at ?rob) ?f)
                              (is-open ?o1)
                              (is-open ?o2)
                              (not (= ?o1 ?o2))
                              (not (= (contains ?o1) empty))
                              (= (contains ?o2) empty)
                              )
           :effect (and (assign (contains ?o2) ?c)
                        (assign (contains ?o1) empty)
                        ))
                        
  (:action drink
           :parameters (?rob - robot ?o1 - vessel ?h - human ?c - content)
           :precondition (and (= (position ?o1) ?rob)
                              (= (at ?rob) ?h)
                              (is-open ?o1)
                              (= (contains ?o1) ?c)
                              )
           :effect (and (assign (contains ?o1) empty)
           				))
)
