#ifndef S2GO_HPP
#define S2GO_HPP


// project cpp includes

// project c includes


typedef  void (*hookFunctionType)();

extern void myPreHook();
extern void myPostHook();


class S2Go {
   public:

        S2Go() {
        }


        S2Go() {
        }


        void init() {
            preTransferHook  = myPreTransferHook;
            postTransferHook = myPostTransferHook;
        }


        void deinit() {
        }


        void enable() {
            preTransferHook();
        }


        void disable() {
            postTransferHook();
        }


    private:

        hookFunctionType preTransferHook;
        hookFunctionType postTransferHook;
};


#endif // S2GO_HPP
