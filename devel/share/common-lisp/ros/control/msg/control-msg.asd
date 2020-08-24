
(cl:in-package :asdf)

(defsystem "control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FourWheeler" :depends-on ("_package_FourWheeler"))
    (:file "_package_FourWheeler" :depends-on ("_package"))
    (:file "FourWheeler" :depends-on ("_package_FourWheeler"))
    (:file "_package_FourWheeler" :depends-on ("_package"))
  ))