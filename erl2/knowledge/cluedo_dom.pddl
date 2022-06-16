(define (domain cluedo_dom)

(:requirements :strips :typing :equality :fluents :disjunctive-preconditions :durative-actions)

(:types
    waypoint
    height
)

(:predicates
    (init_height_random)
    (init_height_adjusted)
    (base_at ?wp - waypoint)
    (arm_at ?h - height)
    (unexplored ?wp - waypoint ?h - height)
    (center ?wp - waypoint)
    (correct_hyp)
    
)

(:functions
    (hints)    ;;number of hints collected
)

(:action adjust_init_height
    :parameters (?h - height)
    :precondition (and (init_height_random))
    :effect (and
        (not (init_height_random))
        (init_height_adjusted)
        (arm_at ?h)
    )
)

(:action goto_waypoint
    :parameters (?from ?to - waypoint)
    :precondition (and (base_at ?from) (init_height_adjusted))
    :effect (and
        (base_at ?to)
        (not (base_at ?from))
    )
)

(:action move_arm
    :parameters (?fromh ?toh - height)
    :precondition (and (arm_at ?fromh))
    :effect (and
        (arm_at ?toh)
        (not (arm_at ?fromh))
    )
)

;;Receive a hint from the current location
(:action get_hint
    :parameters (?wp - waypoint ?h - height)
    :precondition (and (base_at ?wp) (arm_at ?h) (unexplored ?wp ?h))
    :effect (and
        (increase (hints) 1.0)
        (not (unexplored ?wp ?h))
    )
)



(:action check_hypothesis_correct
    :parameters (?wp - waypoint)
    :precondition (and (>= (hints) 3.0) (center ?wp) (base_at ?wp) )
    :effect (and
        (correct_hyp)
        (assign (hints) 0.0)
    )
)



)
