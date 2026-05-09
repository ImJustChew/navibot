import * as React from "react";
import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";
import { cn } from "@/lib/utils";

const buttonVariants = cva(
  "inline-flex items-center justify-center gap-2 whitespace-nowrap font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-blue-400 disabled:pointer-events-none disabled:opacity-50 select-none active:scale-95",
  {
    variants: {
      variant: {
        default:     "bg-blue-600 text-white hover:bg-blue-700 shadow-sm",
        destructive: "bg-red-500  text-white hover:bg-red-600  shadow-sm shadow-red-200",
        outline:     "border border-[#e8e8ed] bg-white text-[#374151] hover:bg-[#f5f5f7] hover:border-[#d1d5db] shadow-sm",
        secondary:   "bg-[#f5f5f7] text-[#374151] hover:bg-[#ebebef]",
        ghost:       "text-[#374151] hover:bg-[#f5f5f7]",
      },
      size: {
        default: "h-9 rounded-lg px-4 py-2 text-sm",
        sm:      "h-8 rounded-md px-3 text-xs",
        lg:      "h-11 rounded-xl px-6 text-sm",
        icon:    "h-12 w-12 rounded-full text-base",
      },
    },
    defaultVariants: { variant: "default", size: "default" },
  },
);

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean;
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : "button";
    return (
      <Comp ref={ref} className={cn(buttonVariants({ variant, size, className }))} {...props} />
    );
  },
);
Button.displayName = "Button";

export { Button, buttonVariants };
