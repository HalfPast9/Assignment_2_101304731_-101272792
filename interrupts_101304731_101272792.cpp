// interrupts.cpp
#include <bits/stdc++.h>
using namespace std;

static inline string trim(const string& s){
    size_t a = s.find_first_not_of(" \t\r\n"); if(a==string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");  return s.substr(a,b-a+1);
}

struct MemoryPartition { int id; int size_mb; string code; };
struct PCB {
    int PID=0, PPID=-1;
    string program="init";
    int size_mb=1;
    int partition=6;
    string state="running";
};

struct TraceLine { string raw, op; vector<string> a; };

struct Sim {
    int t=0, next_pid=1;
    PCB cur{0,-1,"init",1,6,"running"};
    vector<PCB> waitq;
    vector<MemoryPartition> parts{
        {1,40,"free"},{2,25,"free"},{3,15,"free"},
        {4,10,"free"},{5, 8,"free"},{6, 2,"init"}
    };
    vector<int> devtab;               // ISR body by vector index (0-based)
    vector<string> vectab;            // ISR address by vector index (0-based)
    unordered_map<string,int> extsz;  // program -> MB
    string execution, status;
};

static constexpr int MARK_PARTITION_MS = 3;
static constexpr int UPDATE_PCB_MS     = 6;

static vector<int> load_device_table(const string& path){
    vector<int> d; ifstream f(path); string line;
    while(getline(f,line)){ line=trim(line); if(line.empty()) continue;
        for(char& c:line) if(c==',') c=' ';
        stringstream ss(line); int v; if(ss>>v) d.push_back(v);
    } return d;
}
static vector<string> load_vector_table(const string& path){
    vector<string> v; ifstream f(path); string line;
    while(getline(f,line)){ line=trim(line); if(line.empty()) continue; v.push_back(line); }
    return v;
}
static unordered_map<string,int> load_external_files(const string& path){
    unordered_map<string,int> m; ifstream f(path); string line;
    while(getline(f,line)){ line=trim(line); if(line.empty()||line[0]=='#') continue;
        for(char& c:line) if(c==',') c=' ';
        string name; int sz=0; stringstream ss(line);
        if(ss>>name>>sz) m[name]=sz;
    } return m;
}
static vector<TraceLine> load_trace(const string& path){
    vector<TraceLine> V; ifstream f(path); string line;
    while(getline(f,line)){
        line=trim(line); if(line.empty()||line[0]=='#') continue;
        string norm=line; for(char& c:norm) if(c==',') c=' ';
        stringstream ss(norm); vector<string> tok; string w;
        while(ss>>w) tok.push_back(w);
        if(tok.empty()) continue;
        TraceLine tl; tl.raw=line; tl.op=tok[0]; for(char& c:tl.op) c=toupper(c);
        for(size_t i=1;i<tok.size();++i) tl.a.push_back(tok[i]);
        V.push_back(move(tl));
    } return V;
}

static inline string hex4(int x){
    stringstream ss; ss<<uppercase<<hex<<setw(4)<<setfill('0')<<x; return "0x"+ss.str();
}
static void log_prelude(string& out, int& t, int vec, const vector<string>& vectab){
    string mempos = hex4(vec*2);
    string isr    = (vec>=0 && vec<(int)vectab.size()? vectab[vec]:"0X0000");
    out += to_string(t)+", 1, switch to kernel mode\n"; t+=1;
    out += to_string(t)+", 10, context saved\n";         t+=10;
    out += to_string(t)+", 1, find vector "+to_string(vec)+" in memory position "+mempos+"\n"; t+=1;
    out += to_string(t)+", 1, load address "+isr+" into the PC\n"; t+=1;
}
static inline void log_iret(string& out, int& t){ out += to_string(t)+", 1, IRET\n"; t+=1; }

static int find_best_fit_partition(int need_mb, const vector<MemoryPartition>& P){
    int best=-1, best_sz=INT_MAX;
    for(int i=0;i<(int)P.size();++i){
        if(P[i].code=="free" && P[i].size_mb>=need_mb){
            if(P[i].size_mb<best_sz){ best_sz=P[i].size_mb; best=i; }
        }
    } return best;
}

static string format_system_status(int now, const string& current_trace,
                                   const PCB& cur, const vector<PCB>& waitq,
                                   const vector<MemoryPartition>& parts){
    ostringstream out;
    out<<"time: "<<now<<"; current trace: "<<current_trace<<"\n";
    out<<"PID | Program      | Part | Size(MB) | State\n";
    out<<"---------------------------------------------\n";
    out<<setw(3)<<cur.PID<<" | "<<setw(12)<<left<<cur.program<<right<<" | "
       <<setw(4)<<cur.partition<<" | "<<setw(8)<<cur.size_mb<<" | "<<cur.state<<"\n";
    for(const auto& p:waitq){
        out<<setw(3)<<p.PID<<" | "<<setw(12)<<left<<p.program<<right<<" | "
           <<setw(4)<<p.partition<<" | "<<setw(8)<<p.size_mb<<" | "<<p.state<<"\n";
    }
    out<<"\nPartitions:\n#  | Size | Code\n";
    out<<"----------------\n";
    for(const auto& mp:parts){
        out<<setw(2)<<mp.id<<" | "<<setw(4)<<mp.size_mb<<" | "<<mp.code<<"\n";
    }
    out<<"\n";
    return out.str();
}

static void write_output(const string& s, const string& filename){
    ofstream f(filename, ios::out|ios::trunc); f<<s;
}

static void do_syscall(Sim& S, int vec){
    if(vec<0) vec=0;
    log_prelude(S.execution,S.t,vec,S.vectab);
    int body = (vec<(int)S.devtab.size()? S.devtab[vec]:1);
    S.execution += to_string(S.t)+", "+to_string(body)+", SYSCALL ISR\n";
    S.t += body;
    log_iret(S.execution,S.t);
}
static void do_endio(Sim& S, int vec){
    if(vec<0) vec=0;
    log_prelude(S.execution,S.t,vec,S.vectab);
    int body = (vec<(int)S.devtab.size()? S.devtab[vec]:1);
    S.execution += to_string(S.t)+", "+to_string(body)+", ENDIO ISR\n";
    S.t += body;
    log_iret(S.execution,S.t);
}

static void do_fork(Sim& S, int clone_ms, const string& trace_line_for_status){
    int vec=2;
    log_prelude(S.execution,S.t,vec,S.vectab);
    S.execution += to_string(S.t)+", "+to_string(clone_ms)+", cloning the PCB\n";
    S.t += clone_ms;

    PCB parent = S.cur; parent.state="waiting";
    S.waitq.push_back(parent);

    PCB child = parent;
    child.PID   = S.next_pid++;
    child.PPID  = parent.PID;
    child.state = "running";

    // allocate for child's current image (best-fit)
    int need = parent.size_mb;
    int pidx = find_best_fit_partition(need, S.parts);
    if(pidx>=0){
        S.parts[pidx].code = parent.program;
        child.partition    = S.parts[pidx].id;
        child.size_mb      = need;
    }

    S.cur = child;
    S.execution += to_string(S.t)+", 0, scheduler called\n";
    log_iret(S.execution,S.t);

    S.status += format_system_status(S.t, trace_line_for_status, S.cur, S.waitq, S.parts);
}

static void do_exec(Sim& S, const string& prog, int book_ms, const string& trace_line_for_status){
    int vec=3;
    log_prelude(S.execution,S.t,vec,S.vectab);

    if(book_ms<0) book_ms=0;
    S.execution += to_string(S.t)+", "+to_string(book_ms)+", exec bookkeeping\n";
    S.t += book_ms;

    int sz=0; if(auto it=S.extsz.find(prog); it!=S.extsz.end()) sz=it->second;

    int pidx = find_best_fit_partition(sz, S.parts);
    if(pidx<0){
        S.execution += to_string(S.t)+", 1, no free partition fits program\n";
        S.t += 1;
        log_iret(S.execution,S.t);
        S.status += format_system_status(S.t, trace_line_for_status, S.cur, S.waitq, S.parts);
        return;
    }

    int load_ms = 15*sz;
    S.execution += to_string(S.t)+", "+to_string(load_ms)+", loader copy disk->RAM\n";
    S.t += load_ms;

    S.execution += to_string(S.t)+", "+to_string(MARK_PARTITION_MS)+", marking partition as occupied\n";
    S.t += MARK_PARTITION_MS;
    S.parts[pidx].code = prog;

    S.execution += to_string(S.t)+", "+to_string(UPDATE_PCB_MS)+", updating PCB\n";
    S.t += UPDATE_PCB_MS;
    S.cur.program   = prog;
    S.cur.size_mb   = sz;
    S.cur.partition = S.parts[pidx].id;
    S.cur.state     = "running";

    int remaining = S.parts[pidx].size_mb - sz;
    S.execution += to_string(S.t)+", 0, remaining space in partition "
                 + to_string(S.parts[pidx].id) + ": " + to_string(remaining) + " MB\n";

    S.execution += to_string(S.t)+", 0, scheduler called\n";
    log_iret(S.execution,S.t);

    S.status += format_system_status(S.t, trace_line_for_status, S.cur, S.waitq, S.parts);

    // run program sub-trace
    // (missing file is allowed → just no extra work)
    ifstream probe(prog + ".txt");
    if(probe.good()){
        // call into driver for subtrace
        // forward-declared locally:
        extern void simulate_trace_file(Sim&, const string&);
        simulate_trace_file(S, prog + ".txt");
    }
}

static void run_segment(Sim& S, const vector<TraceLine>& TL, long lo, long hi){
    for(long k=lo;k<hi;++k){
        const auto& L = TL[k];
        if(L.op=="IF_CHILD"||L.op=="IF_PARENT"||L.op=="ENDIF") continue;

        if(L.op=="CPU"){
            int ms=L.a.empty()?0:stoi(L.a[0]);
            S.execution += to_string(S.t)+", "+to_string(ms)+", CPU Burst\n";
            S.t += ms;
        }else if(L.op=="SYSCALL"){
            int v=L.a.empty()?0:stoi(L.a[0]); do_syscall(S,v);
        }else if(L.op=="END_IO"){
            int v=L.a.empty()?0:stoi(L.a[0]); do_endio(S,v);
        }else if(L.op=="FORK"){
            int ms=L.a.empty()?0:stoi(L.a[0]); do_fork(S,ms,L.raw);
        }else if(L.op=="EXEC"){
            string prog; int book=0;
            if(L.a.size()==1){ prog=L.a[0]; }
            else if(L.a.size()>=2){ prog=L.a[0]; book=stoi(L.a[1]); }
            do_exec(S,prog,book,L.raw);
        }else{
            // ignore unknown
        }
    }
}

void simulate_trace_file(Sim& S, const string& trace_path){
    ifstream probe(trace_path);
    if(!probe.good()) return;

    vector<TraceLine> TL = load_trace(trace_path);

    long parent_if=-1, parent_end=-1;
    for(long i=0;i<(long)TL.size();++i){
        if(TL[i].op=="IF_PARENT") parent_if=i;
        if(TL[i].op=="ENDIF" && parent_if>=0){ parent_end=i; break; }
    }

    if(parent_if<0 || parent_end<0){
        run_segment(S,TL,0,(long)TL.size());
        return;
    }

    const long parent_block_lo = parent_if+1;
    const long parent_block_hi = parent_end;
    const long tail_lo = parent_end+1;
    const long tail_hi = (long)TL.size();

    // child up to ENDIF (skip parent block)
    for(long i=0;i<parent_end;++i){
        if(i>=parent_block_lo && i<parent_block_hi) continue;
        const auto& L=TL[i];
        if(L.op=="IF_CHILD"||L.op=="IF_PARENT"||L.op=="ENDIF") continue;

        if(L.op=="CPU"){
            int ms=L.a.empty()?0:stoi(L.a[0]);
            S.execution += to_string(S.t)+", "+to_string(ms)+", CPU Burst\n";
            S.t += ms;
        }else if(L.op=="SYSCALL"){
            int v=L.a.empty()?0:stoi(L.a[0]); do_syscall(S,v);
        }else if(L.op=="END_IO"){
            int v=L.a.empty()?0:stoi(L.a[0]); do_endio(S,v);
        }else if(L.op=="FORK"){
            int ms=L.a.empty()?0:stoi(L.a[0]); do_fork(S,ms,L.raw);
        }else if(L.op=="EXEC"){
            string prog; int book=0;
            if(L.a.size()==1){ prog=L.a[0]; }
            else if(L.a.size()>=2){ prog=L.a[0]; book=stoi(L.a[1]); }
            do_exec(S,prog,book,L.raw);
        }
    }

    // child tail (no scheduler here)
    run_segment(S,TL,tail_lo,tail_hi);

    // switch to parent now
    if(!S.waitq.empty()){
        S.cur = S.waitq.back();
        S.waitq.pop_back();
        S.cur.state="running";
        S.execution += to_string(S.t)+", 0, scheduler called\n";
    }

    // parent block then same tail
    run_segment(S,TL,parent_block_lo,parent_block_hi);
    run_segment(S,TL,tail_lo,tail_hi);
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    if(argc!=5){
        cerr<<"ERROR!\nExpected 4 argument, received "<<(argc-1)<<"\n";
        cerr<<"To run the program, do: ./interrutps <your_trace_file.txt> <your_vector_table.txt> <your_device_table.txt> <your_external_files.txt>\n";
        return 1;
    }
    string trace_path=argv[1], vector_table_path=argv[2], device_table_path=argv[3], external_path=argv[4];

    Sim S;
    S.devtab = load_device_table(device_table_path);
    S.vectab = load_vector_table(vector_table_path);
    S.extsz  = load_external_files(external_path);

    simulate_trace_file(S, trace_path);
    write_output(S.execution,"execution.txt");
    write_output(S.status,"system_status.txt");
    return 0;
}
