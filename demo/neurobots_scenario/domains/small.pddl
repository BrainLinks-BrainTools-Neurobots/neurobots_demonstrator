(define (domain neurobots-small)
  (:requirements :adl :object-fluents)
  (:types
   furniture robot - base
   shelf - furniture
   cup glass - transportable
   room   
   color)

  (:constants
   nowhere - base
   uncolored - color)

  (:predicates
   (connected ?l1 ?l2 - room)
   (arm-empty ?r - robot)
   (has-approached ?r - robot ?newbase - base)
   (has-dropped ?dropper - robot ?t - transportable)
   (went-to ?r - robot ?to - room)
   (has-grasped ?rob - robot ?t - transportable))

  (:functions
   (position ?t - transportable) - base
   (at ?r - robot) - base
   (in ?b - base) - room
   (colored ?o - object) - color)

  (:action approach
 	  :parameters (?r - robot ?oldbase ?newbase - base ?rm - room)
 	  :precondition (and (= (at ?r) ?oldbase)
 	  					 (= (in ?r) ?rm)
 	  					 (= (in ?newbase) ?rm)
 	  					 (not (= ?newbase ?r)))
 	  :effect (and (assign (at ?r) ?newbase)
 	  			   (has-approached ?r ?newbase)))

  (:action drop
 	  :parameters (?dropper - robot ?t - transportable ?fu - furniture)
 	  :precondition (and (= (position ?t) ?dropper)
					     (= (at ?dropper) ?fu))
 	  :effect (and (assign (position ?t) ?fu)
	               (arm-empty ?dropper)
	               (has-dropped ?dropper ?t)))

  (:action go
 	  :parameters (?r - robot ?from ?to - room ?oldbase - base)
 	  :precondition (and (= (in ?r) ?from)
					     (= (at ?r) ?oldbase)
					     (connected ?from ?to))
 	  :effect (and (assign (in ?r) ?to)
           		   (assign (at ?r) nowhere)
           		   (went-to ?r ?to)))

  (:action grasp
 	  :parameters (?rob - robot ?t - transportable ?b - base)
 	  :precondition (and (= (at ?rob) ?b)
                         (= (position ?t) ?b)
                         (arm-empty ?rob))
 	  :effect (and (assign (position ?t) ?rob)
                   (not (arm-empty ?rob))
                   (has-grasped ?rob ?t)))
)
