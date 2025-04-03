
(cl:in-package :asdf)

(defsystem "all_om-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "point" :depends-on ("_package_point"))
    (:file "_package_point" :depends-on ("_package"))
    (:file "points" :depends-on ("_package_points"))
    (:file "_package_points" :depends-on ("_package"))
  ))