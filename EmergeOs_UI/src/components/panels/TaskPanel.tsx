"use client";

import React, { useState, useRef, useEffect } from "react";
import { Panel, Group } from "react-resizable-panels";
import { ResizeHandle } from "@/app/page";
import { useDimosStore } from "@/store/useDimosStore";
import {
  Send,
  Bot,
  User,
  Loader2,
  ListTodo,
  PlusCircle,
  ShieldAlert,
  Terminal
} from "lucide-react";

// Message block types for parsing Hermes responses
type BlockType = 'heading' | 'paragraph' | 'code' | 'list' | 'table';

interface MessageBlock {
  type: BlockType;
  content: string;
  language?: string;
  level?: number;           // For headings (1-4)
  items?: string[];         // For lists
  ordered?: boolean;        // For lists
  headers?: string[];       // For tables
  rows?: string[][];        // For tables
}

// Inline markdown rendering: **bold** and `code`
function renderInlineMarkdown(text: string): React.ReactNode {
  const parts = text.split(/(\*\*[^*]+\*\*|`[^`]+`)/);
  return parts.map((part, idx) => {
    if (part.startsWith('**') && part.endsWith('**')) {
      return <strong key={idx} className="font-semibold text-zinc-100">{part.slice(2, -2)}</strong>;
    }
    if (part.startsWith('`') && part.endsWith('`')) {
      return (
        <code key={idx} className="px-1 py-0.5 mx-0.5 text-[10px] font-mono bg-zinc-800 border border-zinc-700/50 rounded text-zinc-300">
          {part.slice(1, -1)}
        </code>
      );
    }
    return part;
  });
}

// Helper to split table row into cells
function splitTableRow(row: string): string[] {
  return row.trim().replace(/^\|/, '').replace(/\|$/, '').split('|').map(c => c.trim());
}

// Check if line is a table separator (|---|---| pattern)
function isTableSeparator(line: string): boolean {
  return /^\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)+\|?$/.test(line.trim());
}

function parseMessageBlocks(content: string): MessageBlock[] {
  const blocks: MessageBlock[] = [];
  const lines = content.split('\n');
  let i = 0;

  while (i < lines.length) {
    const line = lines[i];

    // Fenced code block
    const codeMatch = line.match(/^```(\w*)$/);
    if (codeMatch) {
      const lang = codeMatch[1] || 'text';
      const codeLines: string[] = [];
      i++;
      while (i < lines.length && !lines[i].match(/^```/)) {
        codeLines.push(lines[i]);
        i++;
      }
      blocks.push({ type: 'code', content: codeLines.join('\n'), language: lang });
      i++; // skip closing ```
      continue;
    }

    // Heading (### heading)
    const headingMatch = line.match(/^(#{1,4})\s+(.+)$/);
    if (headingMatch) {
      blocks.push({
        type: 'heading',
        content: headingMatch[2],
        level: headingMatch[1].length
      });
      i++;
      continue;
    }

    // Table detection: line starts with | AND next line is separator
    if (line.trim().startsWith('|') && i + 1 < lines.length && isTableSeparator(lines[i + 1])) {
      const headers = splitTableRow(line);
      const rows: string[][] = [];
      i += 2; // skip header and separator
      while (i < lines.length && lines[i].trim().startsWith('|')) {
        rows.push(splitTableRow(lines[i]));
        i++;
      }
      blocks.push({ type: 'table', content: '', headers, rows });
      continue;
    }

    // List group: consecutive list items
    if (line.match(/^[-*]\s/) || line.match(/^\d+\.\s/)) {
      const ordered = /^\d+\.\s/.test(line);
      const items: string[] = [];

      while (i < lines.length) {
        const currLine = lines[i];
        const unorderedMatch = currLine.match(/^[-*]\s(.+)$/);
        const orderedMatch = currLine.match(/^\d+\.\s(.+)$/);

        if (ordered) {
          if (orderedMatch) {
            items.push(orderedMatch[1]);
            i++;
          } else {
            break;
          }
        } else {
          if (unorderedMatch) {
            items.push(unorderedMatch[1]);
            i++;
          } else {
            break;
          }
        }
      }

      blocks.push({ type: 'list', content: '', items, ordered });
      continue;
    }

    // Empty line - skip but separate paragraphs
    if (!line.trim()) {
      i++;
      continue;
    }

    // Regular paragraph - collect until empty line or special line
    const paraLines: string[] = [];
    while (i < lines.length) {
      const currLine = lines[i];
      if (!currLine.trim()) break;
      if (currLine.match(/^```/) || currLine.match(/^#{1,4}\s/) || currLine.match(/^[-*]\s/) || currLine.match(/^\d+\.\s/)) break;
      if (currLine.trim().startsWith('|')) break;
      paraLines.push(currLine);
      i++;
    }
    if (paraLines.length > 0) {
      blocks.push({ type: 'paragraph', content: paraLines.join('\n') });
    }
  }

  return blocks;
}

function renderBlock(block: MessageBlock, key: number): React.ReactNode {
  switch (block.type) {
    case 'heading':
      const HeadingTag = block.level === 1 || block.level === 2 ? 'h2' : 'h3';
      const headingClass = block.level === 1 || block.level === 2
        ? 'text-sm font-semibold text-zinc-100 mt-2 mb-1'
        : 'text-xs font-semibold text-zinc-100 mt-2 mb-1';
      return React.createElement(HeadingTag, { key, className: headingClass }, renderInlineMarkdown(block.content));

    case 'table':
      if (!block.headers || !block.rows) return null;
      return (
        <div key={key} className="my-2 overflow-x-auto rounded border border-zinc-700/50">
          <table className="w-full text-[10px] border-collapse">
            <thead className="bg-zinc-900/70">
              <tr>
                {block.headers.map((h, i) => (
                  <th key={i} className="px-2 py-1.5 text-left font-semibold text-zinc-300 border-b border-zinc-700/50">
                    {renderInlineMarkdown(h)}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {block.rows.map((row, ri) => (
                <tr key={ri} className="border-b border-zinc-800/50 last:border-b-0">
                  {row.map((cell, ci) => (
                    <td key={ci} className="px-2 py-1.5 text-zinc-400">
                      {renderInlineMarkdown(cell)}
                    </td>
                  ))}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      );

    case 'list':
      if (!block.items || block.items.length === 0) return null;
      const ListTag = block.ordered ? 'ol' : 'ul';
      const listClass = block.ordered
        ? 'my-1 ml-4 list-decimal space-y-0.5'
        : 'my-1 ml-4 list-disc space-y-0.5';
      return React.createElement(
        ListTag,
        { key, className: listClass },
        block.items.map((item, idx) => (
          <li key={idx} className="text-[11px] text-zinc-400/90">
            {renderInlineMarkdown(item)}
          </li>
        ))
      );

    case 'code':
      return (
        <pre key={key} className="text-[10px] font-mono bg-zinc-900/80 border border-zinc-700/50 rounded p-2 overflow-x-auto my-1.5">
          <code>{block.content}</code>
        </pre>
      );

    case 'paragraph':
    default:
      return (
        <p key={key} className="text-xs leading-relaxed my-0.5 whitespace-pre-wrap break-words">
          {renderInlineMarkdown(block.content)}
        </p>
      );
  }
}

const TaskPanel: React.FC = () => {
  const { appLogs, chatMessages, bridgeConnected, dimosLive, hermesConnected, hermesBusy, sendChatMessage, startNewHermesSession, pendingApproval, respondToApproval } = useDimosStore();
  const [inputText, setInputText] = useState("");
  const [mounted, setMounted] = useState(false);
  const chatEndRef = useRef<HTMLDivElement>(null);
  const logEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setMounted(true);
  }, []);

  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chatMessages, pendingApproval]);

  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [appLogs]);

  const handleSend = () => {
    if (inputText.trim() && !hermesBusy) {
      sendChatMessage(inputText);
      setInputText("");
    }
  };

  const handleNewSession = () => {
    if (!hermesConnected || hermesBusy) return;
    startNewHermesSession();
    setInputText("");
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !hermesBusy) {
      handleSend();
    }
  };

  return (
    <Group orientation="vertical" className="h-full">
      {/* Dialogue Summary (EmergeOS Chat) */}
      <Panel defaultSize={60} minSize={30}>
        <section className="h-full bg-card border border-border rounded-lg flex flex-col overflow-hidden">
          <div className="px-4 py-2 border-b border-border bg-zinc-900/50 flex justify-between items-center">
            <h2 className="text-xs font-semibold uppercase tracking-wider text-zinc-400">对话摘要 (EmergeOS)</h2>
            <div className="flex items-center gap-2">
              <button
                onClick={handleNewSession}
                disabled={mounted ? (!hermesConnected || hermesBusy) : true}
                title="新对话"
                className="flex items-center gap-1 px-2 py-1 text-[10px] font-medium text-zinc-400 bg-zinc-800/60 hover:bg-zinc-700/80 hover:text-zinc-200 border border-zinc-700/50 rounded-md transition-all disabled:opacity-30 disabled:cursor-not-allowed disabled:hover:bg-zinc-800/60 disabled:hover:text-zinc-400"
              >
                <PlusCircle size={12} />
                新对话
              </button>
              <Bot size={12} className="text-blue-500" />
            </div>
          </div>

          <div className="flex-1 overflow-y-auto p-4 flex flex-col gap-4 custom-scrollbar">
            {chatMessages.length === 0 && !pendingApproval ? (
              <div className="h-full flex flex-col items-center justify-center text-zinc-600 gap-2 opacity-50">
                <Bot size={32} />
                <p className="text-[10px] uppercase tracking-widest">等待 EmergeOS 对话接入...</p>
              </div>
            ) : (
              chatMessages.map((msg) => {
                // Activity message - compact tool progress
                if (msg.kind === 'activity') {
                  return (
                    <div key={msg.id} className="flex items-center gap-2 py-1 px-2">
                      <Terminal size={11} className="text-zinc-500 shrink-0" />
                      <span className="text-[10px] font-mono text-zinc-400">{msg.content}</span>
                    </div>
                  );
                }

                // Regular message
                const blocks = parseMessageBlocks(msg.content);

                return (
                  <div key={msg.id} className="flex gap-3">
                    <div className={`w-8 h-8 rounded-full flex items-center justify-center shrink-0 border
                      ${msg.role === 'human' ? 'bg-blue-900/40 border-blue-800/50 text-blue-400' : 'bg-zinc-800 border-zinc-700 text-zinc-400'}`}>
                      {msg.role === 'human' ? <User size={14} /> : <Bot size={14} />}
                    </div>
                    <div className="flex flex-col gap-1 max-w-[85%]">
                      <div className="flex items-center gap-2">
                        <span className={`text-[10px] font-semibold ${msg.role === 'human' ? 'text-zinc-300' : 'text-blue-400'}`}>
                          {msg.role === 'human' ? '操作员' : 'EmergeOS'}
                        </span>
                        <span className="text-[9px] text-zinc-600">{msg.timestamp}</span>
                      </div>
                      <div className={`text-xs p-2.5 rounded-r-lg rounded-bl-lg border leading-relaxed
                        ${msg.role === 'human' ? 'text-zinc-400 bg-zinc-800/40 border-zinc-800' : 'text-zinc-300 bg-blue-900/10 border-blue-900/20'}
                        ${msg.pending ? 'opacity-70' : ''}`}>
                        {msg.pending ? (
                          <span className="flex items-center gap-2">
                            <Loader2 size={12} className="animate-spin text-blue-400" />
                            {msg.content}
                          </span>
                        ) : (
                          blocks.map((block, i) => renderBlock(block, i))
                        )}

                        {msg.thought && !msg.pending && (
                          <div className="mt-2 pt-2 border-t border-blue-900/30">
                            <div className="flex items-center gap-1 text-[9px] text-zinc-500 uppercase font-bold mb-1">
                              <ListTodo size={10} /> 思维链 (Thought)
                            </div>
                            <p className="text-[10px] text-zinc-400 italic">
                              {msg.thought}
                            </p>
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                );
              })
            )}

            {/* Dangerous command approval card */}
            {pendingApproval && (
              <div className="flex flex-col gap-2 p-3 rounded-lg border border-orange-700/50 bg-orange-900/10 animate-pulse-once">
                <div className="flex items-center gap-2">
                  <div className="w-8 h-8 rounded-full flex items-center justify-center shrink-0 border bg-orange-900/40 border-orange-700/50 text-orange-400">
                    <ShieldAlert size={14} />
                  </div>
                  <div>
                    <span className="text-[10px] font-semibold text-orange-400 uppercase tracking-wider">危险命令授权请求</span>
                    <p className="text-[9px] text-orange-300/70 mt-0.5">{pendingApproval.description}</p>
                  </div>
                </div>
                <div className="ml-10">
                  <pre className="text-[9px] font-mono text-zinc-400 bg-zinc-900/60 border border-zinc-800 rounded p-2 overflow-x-auto whitespace-pre-wrap break-all">
                    {pendingApproval.command}
                  </pre>
                  <div className="flex flex-wrap gap-1.5 mt-2">
                    <button
                      onClick={() => respondToApproval(pendingApproval.id, 'once')}
                      className="px-2.5 py-1 text-[9px] font-semibold rounded border border-green-700/50 bg-green-900/20 text-green-400 hover:bg-green-800/30 hover:border-green-600 transition-all active:scale-95"
                    >
                      允许一次
                    </button>
                    <button
                      onClick={() => respondToApproval(pendingApproval.id, 'session')}
                      className="px-2.5 py-1 text-[9px] font-semibold rounded border border-blue-700/50 bg-blue-900/20 text-blue-400 hover:bg-blue-800/30 hover:border-blue-600 transition-all active:scale-95"
                    >
                      本次会话允许
                    </button>
                    <button
                      onClick={() => respondToApproval(pendingApproval.id, 'always')}
                      className="px-2.5 py-1 text-[9px] font-semibold rounded border border-purple-700/50 bg-purple-900/20 text-purple-400 hover:bg-purple-800/30 hover:border-purple-600 transition-all active:scale-95"
                    >
                      永久允许
                    </button>
                    <button
                      onClick={() => respondToApproval(pendingApproval.id, 'deny')}
                      className="px-2.5 py-1 text-[9px] font-semibold rounded border border-red-700/50 bg-red-900/20 text-red-400 hover:bg-red-800/30 hover:border-red-600 transition-all active:scale-95"
                    >
                      拒绝
                    </button>
                  </div>
                </div>
              </div>
            )}

            <div ref={chatEndRef} />
          </div>

          <div className="p-4 bg-zinc-900/30 border-t border-border flex items-center gap-3">
            <input
              type="text"
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={hermesBusy ? "EmergeOS 正在思考..." : "输入指令发给 EmergeOS..."}
              disabled={hermesBusy}
              className="flex-1 bg-zinc-900 border border-zinc-800 rounded-lg py-2.5 px-4 text-xs text-zinc-300 focus:outline-none focus:border-blue-600 focus:ring-1 focus:ring-blue-600/20 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
            />
            <button
              onClick={handleSend}
              disabled={mounted ? (!hermesConnected || hermesBusy) : true}
              className="w-10 h-10 bg-blue-600 hover:bg-blue-500 disabled:bg-zinc-800 disabled:text-zinc-600 text-white rounded-lg flex items-center justify-center transition-all shadow-lg shadow-blue-900/20 active:scale-95"
            >
              {hermesBusy ? <Loader2 size={16} className="animate-spin" /> : <Send size={16} />}
            </button>
          </div>
        </section>
      </Panel>

      <ResizeHandle orientation="vertical" />

      {/* Execution Log (Dimos System Logs) */}
      <Panel defaultSize={40} minSize={20}>
        <section className="h-full bg-card border border-border rounded-lg flex flex-col overflow-hidden">
          <div className="px-4 py-2 border-b border-border bg-zinc-900/50 flex justify-between items-center">
            <h2 className="text-xs font-semibold uppercase tracking-wider text-zinc-400">执行日志</h2>
            <span className="text-[9px] text-zinc-600 uppercase font-mono flex items-center gap-1.5">
              <span className={`w-1.5 h-1.5 rounded-full ${dimosLive ? 'bg-green-500 animate-pulse' : bridgeConnected ? 'bg-amber-500' : 'bg-zinc-700'}`}></span>
              {dimosLive ? 'Live' : bridgeConnected ? 'Standby' : 'Offline'}
            </span>
          </div>
          <div className="flex-1 overflow-y-auto p-3 font-mono text-[10px] space-y-1.5 custom-scrollbar bg-black/20">
            {appLogs.map((log) => (
              <div key={log.id} className="flex gap-3 hover:bg-white/5 transition-colors px-1 rounded group">
                <span className="text-zinc-600 shrink-0">{log.timestamp}</span>
                <span className={`
                  ${log.status === 'success' ? 'text-zinc-400' :
                    log.status === 'warning' ? 'text-orange-400' :
                    log.status === 'error' ? 'text-red-400' : 'text-blue-400'}
                `}>[{log.topic?.toUpperCase() || 'SYS'}] {log.message}</span>
              </div>
            ))}
            <div ref={logEndRef} />
            {!dimosLive && appLogs.length === 0 && (
              <div className="text-zinc-700 italic py-2 text-center">消息通道未连接, 等待 EmergeOS 日志流...</div>
            )}
          </div>
        </section>
      </Panel>
    </Group>
  );
};

export default TaskPanel;