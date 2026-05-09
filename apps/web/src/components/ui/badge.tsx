import * as React from "react";
import { cva, type VariantProps } from "class-variance-authority";
import { cn } from "@/lib/utils";

const badgeVariants = cva(
  "inline-flex items-center gap-1.5 rounded-full px-2.5 py-0.5 text-[10px] font-semibold uppercase tracking-[0.1em] transition-colors",
  {
    variants: {
      variant: {
        default:  "bg-blue-100  text-blue-700",
        online:   "bg-green-100 text-green-700",
        offline:  "bg-[#f5f5f7] text-[#9ca3af]",
        warning:  "bg-amber-100 text-amber-700",
        danger:   "bg-red-100   text-red-700",
      },
    },
    defaultVariants: { variant: "default" },
  },
);

export interface BadgeProps
  extends React.HTMLAttributes<HTMLDivElement>,
    VariantProps<typeof badgeVariants> {}

function Badge({ className, variant, ...props }: BadgeProps) {
  return <div className={cn(badgeVariants({ variant }), className)} {...props} />;
}

export { Badge, badgeVariants };
