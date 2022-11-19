typedef struct TriMems //Triangle Membership Function
{
    //char Name[20];
    double Upper;
    double Lower;
    double Peak;
    double degree;

} TriMem;

typedef struct Out_range   //Range of the Output 
{
    BOOLEAN No_range;
    double Min;
    double Max;
}Out_range;

TriMem Create_Member(char name[20],double lower,double peak,double upper); // Create Membership Function

double Get_val(TriMem *Mem,int ArraySize,double Value); // Get highest degree of given value among the Membership functions
   
void Rule(TriMem *Input_set,TriMem *Output_set);// Parse the rule as we want. If we want to parse multi input(s) - output(s) membership functions rule, need to re-code the function as note below 

double Mem_degree(TriMem Mem,double value); // Find the single Membership Function's degree for given value 

double Defuzy(TriMem *OutSet,int ArraySize,Out_range Range); // Defuzzificate function

Out_range Get_range(TriMem *Mem,int ArraySize);  // Determine the out_range of function

TriMem Create_Member(double lower,double peak,double upper)
{
    TriMem Mem;
    Mem.Lower = lower;
    Mem.Peak = peak;
    Mem.Upper = upper;
    //strcpy(Mem.Name, name);
    Mem.degree = 0;
    return Mem;
}

double Mem_degree(TriMem Mem,double value)
{
    if((value<Mem.Lower)||(value>Mem.Upper))
        return 0;
    if(value == Mem.Peak)
        return 1;
    if(value<Mem.Peak)
        return ((value - Mem.Lower)/(Mem.Peak - Mem.Lower));
    else
        return ((Mem.Upper-value)/(Mem.Upper - Mem.Peak));
}

void Rule(TriMem *Input_set,TriMem *Output_set)
{
    Output_set->degree = Input_set->degree;
    // Output_set.degree = Input_set1.degree*Input_set2.degree;    //for 2 input with product composition
    // Output_set.degree = (Input_set1.degree > Input_set2.degree) ? Input_set2.degree : Input_set1.degree;  //2 input with Min composition
}

double Defuzy(TriMem *OutSet,int ArraySize,Out_range Range)
{
    int k = 50;
    double step = (Range.Max - Range.Min)/k;
    int i;

    double ptLeft = 0.0;
    double ptCenter = 0.0;
    double ptRight = 0.0;

    double valLeft = 0.0;
    double valCenter = 0.0;
    double valRight = 0.0;

    double val2Left = 0.0;
    double val2Center = 0.0;
    double val2Right = 0.0;

    double numerator = 0.0;
    double denominator = 0.0;

    for(i=0;i<k;i++)
    {
        if (i == 0)
            {
                ptRight = Range.Min;
                valRight = Get_val(OutSet,ArraySize,ptRight);
                val2Right = ptRight * valRight;
            }

        ptLeft = ptRight;
        ptCenter = Range.Min + step * ((double)i + 0.5);
        ptRight = Range.Min + step * ((double)i + 1);

        valLeft = valRight;
        valCenter = Get_val(OutSet,ArraySize,ptCenter);
        valRight =  Get_val(OutSet,ArraySize,ptRight);

        val2Left = val2Right;
        val2Center = ptCenter * valCenter;
        val2Right = ptRight * valRight;

        numerator += step * (val2Left + 4 * val2Center + val2Right) / 6.0;  //Original 6.0 = 3.0
        denominator += step * (valLeft + 4 * valCenter + valRight) / 6.0;
   }
    return (numerator/denominator);
}

Out_range Get_range(TriMem *Mem,int ArraySize)
{
    Out_range Get_range;
    Get_range.No_range = 1;
    int i;

    for(i=0;i<ArraySize;i++)
    {
        if(Mem[i].degree != 0)
        {
            if(Get_range.No_range)
            {
                Get_range.No_range = 0;
                Get_range.Max = Mem[i].Upper;
                Get_range.Min = Mem[i].Lower;
            }
            else
            {
                Get_range.Max = (Mem[i].Upper>Get_range.Max) ? Mem[i].Upper : Get_range.Max;
                Get_range.Min = (Mem[i].Lower<Get_range.Min) ? Mem[i].Lower : Get_range.Min;
            }
        }
    }
    return Get_range;
}

double Get_val(TriMem *Mem,int ArraySize,double Value)
{
    int i;
    double GVal=0,Val=0;
    TriMem NowVal;
    for(i=0;i<ArraySize;i++)
    {
        NowVal = Mem[i];
        if(NowVal.degree == 0);
        else
        {
            Val = Mem_degree(NowVal,Value);
            Val = (Val>NowVal.degree) ? NowVal.degree : Val ;
            GVal = (Val>GVal) ? Val : GVal;
        }
    }
    return GVal;
}

